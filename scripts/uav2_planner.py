#!/usr/bin/env python
import os
import sys
import json
import math
import heapq
import rospy
import numpy as np
import tf
import csv

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from hector_uav_msgs.srv import EnableMotors
from collections import deque



def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def enable_motors(service_name, enable=True, timeout=10.0):
    rospy.loginfo("Waiting for %s ...", service_name)
    rospy.wait_for_service(service_name, timeout=timeout)
    try:
        srv = rospy.ServiceProxy(service_name, EnableMotors)
        resp = srv(enable)
        rospy.loginfo("enable_motors(%s) -> success=%s", enable, resp.success)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call %s: %s", service_name, str(e))
        return False

def quat_to_rot(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3, dtype=np.float64)

    q = q / n
    x, y, z, w = q

    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R

def make_T(x, y, z, qx, qy, qz, qw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = [x, y, z]
    return T

def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -np.dot(R.T, t)
    return T_inv

class UAV2AStarPlanner(object):
    def __init__(self):
        rospy.init_node("uav2_planner")

        self.saved_map_file = rospy.get_param("~saved_map_file", "")
        self.map_topic = rospy.get_param("~map_topic", "/proj_map")
        self.pose_topic = rospy.get_param("~pose_topic", "/uav2/command/pose")
        self.enable_srv = rospy.get_param("~enable_motors_srv", "/uav2/enable_motors")

        self.world_frame = rospy.get_param("~world_frame", "uav2/world")
        self.base_frame = rospy.get_param("~base_frame", "uav2/base_link")
        self.command_frame = rospy.get_param("~command_frame", "uav2/world")

        self.target_z = float(rospy.get_param("~target_z", 3.0))
        self.hover_before_start = float(rospy.get_param("~hover_before_start", 2.0))
        self.waypoint_reach_time = float(rospy.get_param("~waypoint_reach_time", 2.0))
        self.rate_hz = float(rospy.get_param("~rate", 10.0))

        self.occupancy_threshold = int(rospy.get_param("~occupancy_threshold", 50))
        self.inflation_radius_cells = int(rospy.get_param("~inflation_radius_cells", 2))
        self.path_sampling_step = int(rospy.get_param("~path_sampling_step", 5))

        self.map_origin_file = rospy.get_param("~map_origin_file")
        self.tag_map_file = rospy.get_param("~tag_map_file")

        self.grid_msg = None
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)

        self.T_world_map = None
        self.T_map_world = None

        self.results_dir = rospy.get_param("~results_dir", os.path.expanduser("~/simulation_results"))
        self.run_id = rospy.get_param("~run_id", "")
        self.foi_gt_x = float(rospy.get_param("~foi_gt_x", 0.0))
        self.foi_gt_y = float(rospy.get_param("~foi_gt_y", 0.0))
        self.foi_gt_z = float(rospy.get_param("~foi_gt_z", 0.0))

        if not self.run_id:
            self.run_id = "run_%d" % int(rospy.Time.now().to_sec())

        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)

        self.summary_csv = os.path.join(self.results_dir, "run_summary_uav2.csv")
        self.detail_json = os.path.join(self.results_dir, "%s_uav2_metrics.json" % self.run_id)

        self.start_world = None
        self.goal_world = None
        self.foi_hover_world = None
        self.final_home_world = None
        self.mission_start_time = None
        self.mission_end_time = None

    def save_uav2_metrics(self):
        def dist2d(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        hover_error = float("nan")
        home_error = float("nan")
        foi_gt_error = float("nan")
        mission_time = float("nan")
        success = 0

        if self.foi_hover_world is not None and self.goal_world is not None:
            hover_error = dist2d(self.foi_hover_world, self.goal_world)

        if self.final_home_world is not None and self.start_world is not None:
            home_error = dist2d(self.final_home_world, self.start_world)

        if self.foi_hover_world is not None:
            foi_gt_error = math.sqrt(
                (self.foi_hover_world[0] - self.foi_gt_x) ** 2 +
                (self.foi_hover_world[1] - self.foi_gt_y) ** 2 +
                (self.target_z - self.foi_gt_z) ** 2
            )

        if self.mission_start_time is not None and self.mission_end_time is not None:
            mission_time = self.mission_end_time - self.mission_start_time

        if not math.isnan(hover_error) and not math.isnan(home_error):
            success = 1

        metrics = {
            "run_id": self.run_id,
            "uav2_start_x_world_m": self.start_world[0] if self.start_world else float("nan"),
            "uav2_start_y_world_m": self.start_world[1] if self.start_world else float("nan"),
            "uav2_goal_x_world_m": self.goal_world[0] if self.goal_world else float("nan"),
            "uav2_goal_y_world_m": self.goal_world[1] if self.goal_world else float("nan"),
            "uav2_foi_hover_x_world_m": self.foi_hover_world[0] if self.foi_hover_world else float("nan"),
            "uav2_foi_hover_y_world_m": self.foi_hover_world[1] if self.foi_hover_world else float("nan"),
            "uav2_final_home_x_world_m": self.final_home_world[0] if self.final_home_world else float("nan"),
            "uav2_final_home_y_world_m": self.final_home_world[1] if self.final_home_world else float("nan"),
            "uav2_hover_error_to_commanded_foi_m": hover_error,
            "uav2_final_home_error_m": home_error,
            "uav2_hover_error_to_gt_foi_m": foi_gt_error,
            "uav2_mission_time_s": mission_time,
            "uav2_success": success
        }

        with open(self.detail_json, "w") as f:
            json.dump(metrics, f, indent=2)

        header = [
            "run_id",
            "uav2_start_x_world_m",
            "uav2_start_y_world_m",
            "uav2_goal_x_world_m",
            "uav2_goal_y_world_m",
            "uav2_foi_hover_x_world_m",
            "uav2_foi_hover_y_world_m",
            "uav2_final_home_x_world_m",
            "uav2_final_home_y_world_m",
            "uav2_hover_error_to_commanded_foi_m",
            "uav2_final_home_error_m",
            "uav2_hover_error_to_gt_foi_m",
            "uav2_mission_time_s",
            "uav2_success"
        ]

        file_exists = os.path.exists(self.summary_csv)
        with open(self.summary_csv, "a") as f:
            writer = csv.DictWriter(f, fieldnames=header)
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)

        rospy.loginfo("Saved UAV2 metrics to %s", self.summary_csv)

    def save_uav2_metrics(self):
        def dist2d(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        hover_error = float("nan")
        home_error = float("nan")
        foi_gt_error = float("nan")
        mission_time = float("nan")
        success = 0

        if self.foi_hover_world is not None and self.goal_world is not None:
            hover_error = dist2d(self.foi_hover_world, self.goal_world)

        if self.final_home_world is not None and self.start_world is not None:
            home_error = dist2d(self.final_home_world, self.start_world)

        if self.foi_hover_world is not None:
            foi_gt_error = math.sqrt(
                (self.foi_hover_world[0] - self.foi_gt_x) ** 2 +
                (self.foi_hover_world[1] - self.foi_gt_y) ** 2 +
                (self.target_z - self.foi_gt_z) ** 2
            )

        if self.mission_start_time is not None and self.mission_end_time is not None:
            mission_time = self.mission_end_time - self.mission_start_time

        if not math.isnan(hover_error) and not math.isnan(home_error):
            success = 1

        metrics = {
            "run_id": self.run_id,
            "uav2_start_x_world_m": self.start_world[0] if self.start_world else float("nan"),
            "uav2_start_y_world_m": self.start_world[1] if self.start_world else float("nan"),
            "uav2_goal_x_world_m": self.goal_world[0] if self.goal_world else float("nan"),
            "uav2_goal_y_world_m": self.goal_world[1] if self.goal_world else float("nan"),
            "uav2_foi_hover_x_world_m": self.foi_hover_world[0] if self.foi_hover_world else float("nan"),
            "uav2_foi_hover_y_world_m": self.foi_hover_world[1] if self.foi_hover_world else float("nan"),
            "uav2_final_home_x_world_m": self.final_home_world[0] if self.final_home_world else float("nan"),
            "uav2_final_home_y_world_m": self.final_home_world[1] if self.final_home_world else float("nan"),
            "uav2_hover_error_to_commanded_foi_m": hover_error,
            "uav2_final_home_error_m": home_error,
            "uav2_hover_error_to_gt_foi_m": foi_gt_error,
            "uav2_mission_time_s": mission_time,
            "uav2_success": success
        }

        with open(self.detail_json, "w") as f:
            json.dump(metrics, f, indent=2)

        header = [
            "run_id",
            "uav2_start_x_world_m",
            "uav2_start_y_world_m",
            "uav2_goal_x_world_m",
            "uav2_goal_y_world_m",
            "uav2_foi_hover_x_world_m",
            "uav2_foi_hover_y_world_m",
            "uav2_final_home_x_world_m",
            "uav2_final_home_y_world_m",
            "uav2_hover_error_to_commanded_foi_m",
            "uav2_final_home_error_m",
            "uav2_hover_error_to_gt_foi_m",
            "uav2_mission_time_s",
            "uav2_success"
        ]

        file_exists = os.path.exists(self.summary_csv)
        with open(self.summary_csv, "a") as f:
            writer = csv.DictWriter(f, fieldnames=header)
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)

        rospy.loginfo("Saved UAV2 metrics to %s", self.summary_csv)

    def load_saved_map(self, json_file):
        if not os.path.exists(json_file):
            raise RuntimeError("Saved map file does not exist: %s" % json_file)

        with open(json_file, "r") as f:
            d = json.load(f)

        msg = OccupancyGrid()
        msg.header.frame_id = d["frame_id"]

        msg.info = MapMetaData()
        msg.info.resolution = d["resolution"]
        msg.info.width = d["width"]
        msg.info.height = d["height"]

        msg.info.origin = Pose()
        msg.info.origin.position.x = d["origin"]["x"]
        msg.info.origin.position.y = d["origin"]["y"]
        msg.info.origin.position.z = d["origin"]["z"]
        msg.info.origin.orientation.x = d["origin"]["qx"]
        msg.info.origin.orientation.y = d["origin"]["qy"]
        msg.info.origin.orientation.z = d["origin"]["qz"]
        msg.info.origin.orientation.w = d["origin"]["qw"]

        msg.data = d["data"]
        self.grid_msg = msg

        rospy.loginfo("Loaded saved occupancy map: %s", json_file)
        rospy.loginfo("Map size: %d x %d, resolution=%.3f, frame_id=%s",
                      msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id)

    def wait_for_live_map(self, timeout=15.0):
        rospy.loginfo("Waiting for occupancy map on %s ...", self.map_topic)
        msg = rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=timeout)
        self.grid_msg = msg
        rospy.loginfo("Live map received: %d x %d, resolution=%.3f, frame_id=%s",
                      msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id)

    def get_uav2_world_pose(self):
        rospy.loginfo("Waiting for TF: %s -> %s", self.world_frame, self.base_frame)
        self.tf_listener.waitForTransform(
            self.world_frame, self.base_frame, rospy.Time(0), rospy.Duration(10.0)
        )
        trans, rot = self.tf_listener.lookupTransform(
            self.world_frame, self.base_frame, rospy.Time(0)
        )
        return trans, rot

    def load_T_world_map(self):
        if not os.path.exists(self.map_origin_file):
            raise RuntimeError("Missing map_origin_file: %s" % self.map_origin_file)

        with open(self.map_origin_file, "r") as f:
            origin = json.load(f)

        self.T_world_map = make_T(
            origin["x"], origin["y"], origin["z"],
            origin["qx"], origin["qy"], origin["qz"], origin["qw"]
        )
        self.T_map_world = invert_T(self.T_world_map)

        rospy.loginfo("Loaded map origin transform (map -> world) from %s", self.map_origin_file)
        rospy.loginfo("T_world_map translation: (%.3f, %.3f, %.3f)",
                      origin["x"], origin["y"], origin["z"])

    def load_goal_map_from_file(self):
        if not os.path.exists(self.tag_map_file):
            raise RuntimeError("Missing tag_map_file: %s" % self.tag_map_file)

        with open(self.tag_map_file, "r") as f:
            tag_map = json.load(f)

        goal_x = float(tag_map["x"])
        goal_y = float(tag_map["y"])
        goal_z = float(tag_map.get("z", 0.0))

        rospy.loginfo("FOI in map frame: (%.3f, %.3f, %.3f)", goal_x, goal_y, goal_z)
        return goal_x, goal_y

    def transform_world_to_map_xy(self, x, y, z=0.0):
        p_world = np.array([x, y, z, 1.0], dtype=np.float64)
        p_map = np.dot(self.T_map_world, p_world)
        return float(p_map[0]), float(p_map[1]), float(p_map[2])

    def transform_map_to_world_xy(self, x, y, z=0.0):
        p_map = np.array([x, y, z, 1.0], dtype=np.float64)
        p_world = np.dot(self.T_world_map, p_map)
        return float(p_world[0]), float(p_world[1]), float(p_world[2])

    def map_metric_to_grid(self, x, y):
        info = self.grid_msg.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution

        gx = int(math.floor((x - ox) / res))
        gy = int(math.floor((y - oy) / res))
        return gx, gy

    def grid_to_map_metric(self, gx, gy):
        info = self.grid_msg.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution

        x = ox + (gx + 0.5) * res
        y = oy + (gy + 0.5) * res
        return x, y
    def interpolate_line_map(self, x0, y0, x1, y1, step=0.25):
        dx = x1 - x0
        dy = y1 - y0
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return [(x1, y1)]

        n = max(1, int(math.ceil(dist / step)))
        pts = []
        for i in range(1, n + 1):
            t = float(i) / float(n)
            # smoothstep interpolation
            ts = t * t * (3.0 - 2.0 * t)
            xi = x0 + dx * ts
            yi = y0 + dy * ts
            pts.append((xi, yi))
        return pts

    def fly_map_waypoints(self, map_points, z, label="Path"):
        rospy.loginfo("%s: %d map-frame waypoints", label, len(map_points))

        for i, (map_x, map_y) in enumerate(map_points):
            world_x, world_y, _ = self.transform_map_to_world_xy(map_x, map_y, 0.0)

            rospy.loginfo("%s waypoint %d/%d -> map (%.3f, %.3f) -> world (%.3f, %.3f, %.3f)",
                          label, i + 1, len(map_points), map_x, map_y, world_x, world_y, z)

            self.publish_pose_for_duration(world_x, world_y, z,
                                           yaw=0.0, duration=self.waypoint_reach_time)

    def inflate_map(self, occ, radius):
        if radius <= 0:
            return occ

        h, w = occ.shape
        inflated = occ.copy()
        occupied_cells = np.argwhere(occ == 1)

        for cy, cx in occupied_cells:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    ny = cy + dy
                    nx = cx + dx
                    if 0 <= ny < h and 0 <= nx < w:
                        if dx*dx + dy*dy <= radius*radius:
                            inflated[ny, nx] = 1

        return inflated

    def build_binary_occupancy(self):
        info = self.grid_msg.info
        w = info.width
        h = info.height

        raw = np.array(self.grid_msg.data, dtype=np.int16).reshape((h, w))
        raw = np.where(raw < 0, 100, raw)  # unknown -> occupied
        occ = (raw >= self.occupancy_threshold).astype(np.uint8)
        occ = self.inflate_map(occ, self.inflation_radius_cells)

        return occ

    def in_bounds(self, x, y, w, h):
        return 0 <= x < w and 0 <= y < h

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, occ, start, goal):
        h, w = occ.shape

        if not self.in_bounds(start[0], start[1], w, h):
            raise RuntimeError("Start is outside occupancy map: %s" % (str(start),))
        if not self.in_bounds(goal[0], goal[1], w, h):
            raise RuntimeError("Goal is outside occupancy map: %s" % (str(goal),))

        if occ[start[1], start[0]] != 0:
            raise RuntimeError("Start cell is occupied: %s" % (str(start),))
        if occ[goal[1], goal[0]] != 0:
            raise RuntimeError("Goal cell is occupied: %s" % (str(goal),))

        neighbors = [
            ( 1,  0, 1.0), (-1,  0, 1.0),
            ( 0,  1, 1.0), ( 0, -1, 1.0),
            ( 1,  1, math.sqrt(2)), ( 1, -1, math.sqrt(2)),
            (-1,  1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ]

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, step_cost in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                nxt = (nx, ny)

                if not self.in_bounds(nx, ny, w, h):
                    continue
                if occ[ny, nx] != 0:
                    continue

                tentative_g = g_score[current] + step_cost

                if nxt not in g_score or tentative_g < g_score[nxt]:
                    came_from[nxt] = current
                    g_score[nxt] = tentative_g
                    f = tentative_g + self.heuristic(nxt, goal)
                    heapq.heappush(open_heap, (f, nxt))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def downsample_path(self, path, step):
        if len(path) <= 2 or step <= 1:
            return path

        reduced = [path[0]]
        i = step
        while i < len(path) - 1:
            reduced.append(path[i])
            i += step
        reduced.append(path[-1])
        return reduced

    def publish_pose_for_duration(self, x, y, z, yaw=0.0, duration=1.0):
        msg = PoseStamped()
        msg.header.frame_id = self.command_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = yaw_to_quat(yaw)

        rate = rospy.Rate(self.rate_hz)
        ticks = max(1, int(duration * self.rate_hz))

        for _ in range(ticks):
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()

    def nearest_free_cell(self, occ, start, max_radius=20):
        h, w = occ.shape
        sx, sy = start

        if self.in_bounds(sx, sy, w, h) and occ[sy, sx] == 0:
            return start

        visited = set()
        q = deque()
        q.append((sx, sy))
        visited.add((sx, sy))

        while q:
            x, y = q.popleft()

            if abs(x - sx) > max_radius or abs(y - sy) > max_radius:
                continue

            if self.in_bounds(x, y, w, h) and occ[y, x] == 0:
                return (x, y)

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    q.append((nx, ny))

        return None
    
    def run(self):
        if self.saved_map_file:
            self.load_saved_map(self.saved_map_file)
        else:
            self.wait_for_live_map(timeout=20.0)

        self.load_T_world_map()

        ok = enable_motors(self.enable_srv, True, timeout=10.0)
        if not ok:
            rospy.logwarn("Motors not enabled; continuing anyway.")

        # Initial UAV2 pose in world
        trans, rot = self.get_uav2_world_pose()
        start_world_x = float(trans[0])
        start_world_y = float(trans[1])
        start_world_z = float(trans[2])

        self.start_world = (start_world_x, start_world_y)
        self.mission_start_time = rospy.Time.now().to_sec()

        # Goal in map frame
        goal_map_x, goal_map_y = self.load_goal_map_from_file()

        # Start in map frame
        start_map_x, start_map_y, start_map_z = self.transform_world_to_map_xy(
            start_world_x, start_world_y, start_world_z
        )

        # Goal also expressed in world just for logging
        goal_world_x, goal_world_y, goal_world_z = self.transform_map_to_world_xy(
            goal_map_x, goal_map_y, 0.0
        )
        
        self.goal_world = (goal_world_x, goal_world_y)

        rospy.loginfo("UAV2 world start:   (%.3f, %.3f, %.3f)",
                      start_world_x, start_world_y, start_world_z)
        rospy.loginfo("UAV2 start in map:  (%.3f, %.3f, %.3f)",
                      start_map_x, start_map_y, start_map_z)
        rospy.loginfo("FOI goal in map:    (%.3f, %.3f)", goal_map_x, goal_map_y)
        rospy.loginfo("FOI approx in world:(%.3f, %.3f, %.3f)",
                      goal_world_x, goal_world_y, goal_world_z)

        # Straight-line map-frame waypoints from start to goal
        outbound_map_points = self.interpolate_line_map(
            start_map_x, start_map_y, goal_map_x, goal_map_y, step=0.25
        )

        rospy.loginfo("Outbound straight-line path has %d map waypoints",
                      len(outbound_map_points))

        # Stabilize and take off
        rospy.loginfo("Stabilizing at current pose before takeoff ...")
        self.publish_pose_for_duration(start_world_x, start_world_y, start_world_z,
                                       yaw=0.0, duration=1.0)

        rospy.loginfo("Taking off gradually to target altitude %.2f ...", self.target_z)
        steps = 30
        for i in range(steps):
            z = start_world_z + (self.target_z - start_world_z) * float(i + 1) / float(steps)
            self.publish_pose_for_duration(start_world_x, start_world_y, z,
                                           yaw=0.0, duration=0.1)

        rospy.loginfo("Hovering at target altitude before outbound flight ...")
        self.publish_pose_for_duration(start_world_x, start_world_y, self.target_z,
                                       yaw=0.0, duration=2.0)

        # Refresh actual pose after takeoff
        trans2, rot2 = self.get_uav2_world_pose()
        current_world_x = float(trans2[0])
        current_world_y = float(trans2[1])
        current_world_z = float(trans2[2])

        rospy.loginfo("Post-takeoff UAV2 world pose: (%.3f, %.3f, %.3f)",
                      current_world_x, current_world_y, current_world_z)

        # Recompute map start after takeoff drift
        current_map_x, current_map_y, current_map_z = self.transform_world_to_map_xy(
            current_world_x, current_world_y, current_world_z
        )

        # Rebuild outbound line from actual current position
        outbound_map_points = self.interpolate_line_map(
            current_map_x, current_map_y, goal_map_x, goal_map_y, step=0.25
        )

        self.fly_map_waypoints(outbound_map_points, self.target_z, label="Outbound")

        rospy.loginfo("Hovering at FOI ...")
        self.publish_pose_for_duration(goal_world_x, goal_world_y, self.target_z,
                                       yaw=0.0, duration=2.0)
        
        trans_hover, rot_hover = self.get_uav2_world_pose()
        self.foi_hover_world = (float(trans_hover[0]), float(trans_hover[1]))
        
        # Return line in map frame: goal back to original start
        return_map_points = self.interpolate_line_map(
            goal_map_x, goal_map_y, start_map_x, start_map_y, step=0.25
        )

        self.fly_map_waypoints(return_map_points, self.target_z, label="Return")

        rospy.loginfo("Hovering at home before landing ...")
        self.publish_pose_for_duration(start_world_x, start_world_y, self.target_z,
                                       yaw=0.0, duration=2.0)

        rospy.loginfo("Landing gradually ...")
        land_steps = 30
        for i in range(land_steps):
            z = self.target_z + (start_world_z - self.target_z) * float(i + 1) / float(land_steps)
            self.publish_pose_for_duration(start_world_x, start_world_y, z,
                                           yaw=0.0, duration=0.1)

        rospy.loginfo("Holding landed pose ...")
        self.publish_pose_for_duration(start_world_x, start_world_y, start_world_z,
                                       yaw=0.0, duration=1.0)
        
        trans_home, rot_home = self.get_uav2_world_pose()
        self.final_home_world = (float(trans_home[0]), float(trans_home[1]))
        self.mission_end_time = rospy.Time.now().to_sec()

        self.save_uav2_metrics()

        rospy.loginfo("UAV2 straight-line map-frame mission complete.")
if __name__ == "__main__":
    try:
        planner = UAV2AStarPlanner()
        planner.run()
    except Exception as e:
        rospy.logerr("uav2_planner failed: %s", str(e))
        sys.exit(1)