#!/usr/bin/env python
import os
import sys
import json
import math
import heapq
import rospy
import numpy as np
import tf

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
        rospy.init_node("uav2_astar_planner")

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

        trans, rot = self.get_uav2_world_pose()
        start_world_x = float(trans[0])
        start_world_y = float(trans[1])
        start_world_z = float(trans[2])

        goal_map_x, goal_map_y = self.load_goal_map_from_file()

        start_map_x, start_map_y, start_map_z = self.transform_world_to_map_xy(
            start_world_x, start_world_y, start_world_z
        )

        goal_world_x, goal_world_y, goal_world_z = self.transform_map_to_world_xy(
            goal_map_x, goal_map_y, 0.0
        )

        rospy.loginfo("UAV2 world start:   (%.3f, %.3f, %.3f)",
                      start_world_x, start_world_y, start_world_z)
        rospy.loginfo("UAV2 start in map:  (%.3f, %.3f, %.3f)",
                      start_map_x, start_map_y, start_map_z)
        rospy.loginfo("FOI goal in map:    (%.3f, %.3f)", goal_map_x, goal_map_y)
        rospy.loginfo("FOI approx in world:(%.3f, %.3f, %.3f)",
                      goal_world_x, goal_world_y, goal_world_z)

        occ = self.build_binary_occupancy()

        start_cell = self.map_metric_to_grid(start_map_x, start_map_y)

        free_start = self.nearest_free_cell(occ, start_cell, max_radius=20)
        if free_start is None:
            raise RuntimeError("Could not find a nearby free start cell from %s" % (str(start_cell),))

        if free_start != start_cell:
            rospy.logwarn("Start cell %s occupied, using nearest free cell %s",
                        str(start_cell), str(free_start))
        start_cell = free_start

        goal_cell = self.map_metric_to_grid(goal_map_x, goal_map_y)

        rospy.loginfo("Start cell: %s", str(start_cell))
        rospy.loginfo("Goal  cell: %s", str(goal_cell))

        path_cells = self.astar(occ, start_cell, goal_cell)
        if path_cells is None:
            raise RuntimeError("A* could not find a path")

        rospy.loginfo("Raw path length: %d cells", len(path_cells))

        path_cells = self.downsample_path(path_cells, self.path_sampling_step)
        rospy.loginfo("Downsampled path length: %d waypoints", len(path_cells))

        rospy.loginfo("Stabilizing at current pose before takeoff ...")
        self.publish_pose_for_duration(start_world_x, start_world_y, start_world_z,
                                    yaw=0.0, duration=1.0)

        rospy.loginfo("Taking off gradually to target altitude %.2f ...", self.target_z)
        steps = 30
        for i in range(steps):
            z = start_world_z + (self.target_z - start_world_z) * float(i + 1) / float(steps)
            self.publish_pose_for_duration(start_world_x, start_world_y, z,
                                        yaw=0.0, duration=0.1)

        rospy.loginfo("Hovering at target altitude before path following ...")
        self.publish_pose_for_duration(start_world_x, start_world_y, self.target_z,
                                    yaw=0.0, duration=2.0)

        trans2, rot2 = self.get_uav2_world_pose()
        current_world_x = float(trans2[0])
        current_world_y = float(trans2[1])
        current_world_z = float(trans2[2])

        rospy.loginfo("Post-takeoff UAV2 world pose: (%.3f, %.3f, %.3f)", current_world_x, current_world_y, current_world_z)
        
        for i, cell in enumerate(path_cells):
            map_x, map_y = self.grid_to_map_metric(cell[0], cell[1])
            world_x, world_y, _ = self.transform_map_to_world_xy(map_x, map_y, 0.0)

            rospy.loginfo("Waypoint %d/%d -> map (%.3f, %.3f) -> world (%.3f, %.3f, %.3f)",
                          i + 1, len(path_cells), map_x, map_y, world_x, world_y, self.target_z)

            self.publish_pose_for_duration(world_x, world_y, self.target_z,
                                           yaw=0.0, duration=self.waypoint_reach_time)

        rospy.loginfo("UAV2 A* mission complete.")

if __name__ == "__main__":
    try:
        planner = UAV2AStarPlanner()
        planner.run()
    except Exception as e:
        rospy.logerr("uav2_astar_planner failed: %s", str(e))
        sys.exit(1)