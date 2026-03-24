#!/usr/bin/env python
import os
import csv
import json
import math
import rospy
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool

class MissionMetricsLogger(object):
    def __init__(self):
        rospy.init_node("mission_metrics_logger")

        self.results_dir = rospy.get_param("~results_dir", os.path.expanduser("~/simulation_results"))
        self.run_id = rospy.get_param("~run_id", "")
        self.foi_gt_x = float(rospy.get_param("~foi_gt_x", 0.0))
        self.foi_gt_y = float(rospy.get_param("~foi_gt_y", 0.0))
        self.foi_gt_z = float(rospy.get_param("~foi_gt_z", 0.0))

        self.map_roi_x_min = float(rospy.get_param("~map_roi_x_min", -20.0))
        self.map_roi_x_max = float(rospy.get_param("~map_roi_x_max", 20.0))
        self.map_roi_y_min = float(rospy.get_param("~map_roi_y_min", -15.0))
        self.map_roi_y_max = float(rospy.get_param("~map_roi_y_max", 15.0))

        self.uav1_gt_topic = rospy.get_param("~uav1_gt_topic", "/uav1/ground_truth/state")
        self.uav1_est_topic = rospy.get_param("~uav1_est_topic", "/uav1/odom")
        self.map_topic = rospy.get_param("~map_topic", "/proj_map")
        self.finished_topic = rospy.get_param("~finished_topic", "/lawnmower_finished")

        self.foi_file = rospy.get_param(
            "~foi_file",
            os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export/apriltag_median_map.json")
        )
        self.map_origin_file = rospy.get_param(
            "~map_origin_file",
            os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export/map_origin.json")
        )

        if not self.run_id:
            self.run_id = rospy.Time.now().to_sec()
            self.run_id = "run_%d" % int(self.run_id)

        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)

        self.summary_csv = os.path.join(self.results_dir, "run_summary.csv")
        self.detail_json = os.path.join(self.results_dir, "%s_uav1_metrics.json" % self.run_id)

        self.uav1_gt = []
        self.uav1_est = []
        self.latest_map = None

        rospy.Subscriber(self.uav1_gt_topic, Odometry, self.uav1_gt_cb, queue_size=200)
        rospy.Subscriber(self.uav1_est_topic, Odometry, self.uav1_est_cb, queue_size=200)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb, queue_size=10)
        rospy.Subscriber(self.finished_topic, Bool, self.finished_cb, queue_size=1)

        rospy.loginfo("mission_metrics_logger ready. run_id=%s", self.run_id)
        rospy.spin()

    def uav1_gt_cb(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        self.uav1_gt.append((t, p.x, p.y, p.z))

    def uav1_est_cb(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        self.uav1_est.append((t, p.x, p.y, p.z))

    def map_cb(self, msg):
        self.latest_map = msg

    def finished_cb(self, msg):
        if not msg.data:
            return

        rospy.loginfo("Lawnmower finished detected. Computing UAV1 metrics...")
        try:
            metrics = self.compute_metrics()
            self.save_metrics(metrics)
            rospy.loginfo("UAV1 metrics saved.")
        except Exception as e:
            rospy.logerr("Failed computing/saving metrics: %s", str(e))

    def compute_metrics(self):
        rms, mean_e, max_e, final_drift = self.compute_uav1_pose_errors()
        map_completeness = self.compute_map_completeness()
        foi_metrics = self.compute_foi_metrics()

        result = {
            "run_id": self.run_id,
            "uav1_rms_position_error_m": rms,
            "uav1_mean_position_error_m": mean_e,
            "uav1_max_position_error_m": max_e,
            "uav1_final_drift_m": final_drift,
            "map_completeness_percent": map_completeness,
            "foi_detected_x_world_m": foi_metrics["detected_x_world"],
            "foi_detected_y_world_m": foi_metrics["detected_y_world"],
            "foi_detected_z_world_m": foi_metrics["detected_z_world"],
            "foi_gt_x_world_m": self.foi_gt_x,
            "foi_gt_y_world_m": self.foi_gt_y,
            "foi_gt_z_world_m": self.foi_gt_z,
            "foi_position_error_m": foi_metrics["foi_error"],
            "foi_detection_success": foi_metrics["success"]
        }
        return result

    def compute_uav1_pose_errors(self):
        if len(self.uav1_gt) < 2 or len(self.uav1_est) < 2:
            rospy.logwarn("Not enough UAV1 samples for pose error.")
            return float("nan"), float("nan"), float("nan"), float("nan")

        gt_times = np.array([x[0] for x in self.uav1_gt], dtype=np.float64)
        gt_xyz = np.array([[x[1], x[2], x[3]] for x in self.uav1_gt], dtype=np.float64)

        est_times = np.array([x[0] for x in self.uav1_est], dtype=np.float64)
        est_xyz = np.array([[x[1], x[2], x[3]] for x in self.uav1_est], dtype=np.float64)

        common_t0 = max(gt_times[0], est_times[0])
        common_t1 = min(gt_times[-1], est_times[-1])

        if common_t1 <= common_t0:
            rospy.logwarn("No overlapping time range for UAV1 GT and estimate.")
            return float("nan"), float("nan"), float("nan"), float("nan")

        sample_times = np.linspace(common_t0, common_t1, num=300)

        gt_interp = np.zeros((len(sample_times), 3), dtype=np.float64)
        est_interp = np.zeros((len(sample_times), 3), dtype=np.float64)

        for k in range(3):
            gt_interp[:, k] = np.interp(sample_times, gt_times, gt_xyz[:, k])
            est_interp[:, k] = np.interp(sample_times, est_times, est_xyz[:, k])

        errors = np.linalg.norm(gt_interp - est_interp, axis=1)
        rms = float(np.sqrt(np.mean(errors ** 2)))
        mean_e = float(np.mean(errors))
        max_e = float(np.max(errors))
        final_drift = float(errors[-1])

        return rms, mean_e, max_e, final_drift

    def compute_map_completeness(self):
        if self.latest_map is None:
            rospy.logwarn("No occupancy grid received.")
            return float("nan")

        info = self.latest_map.info
        width = info.width
        height = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        data = np.array(self.latest_map.data, dtype=np.int16).reshape((height, width))

        x0 = int(math.floor((self.map_roi_x_min - ox) / res))
        x1 = int(math.ceil((self.map_roi_x_max - ox) / res))
        y0 = int(math.floor((self.map_roi_y_min - oy) / res))
        y1 = int(math.ceil((self.map_roi_y_max - oy) / res))

        x0 = max(0, min(width - 1, x0))
        x1 = max(0, min(width, x1))
        y0 = max(0, min(height - 1, y0))
        y1 = max(0, min(height, y1))

        roi = data[y0:y1, x0:x1]
        if roi.size == 0:
            rospy.logwarn("ROI is empty in occupancy grid.")
            return float("nan")

        known = np.sum(roi != -1)
        total = roi.size
        coverage = 100.0 * float(known) / float(total)
        return coverage

    def compute_foi_metrics(self):
        result = {
            "detected_x_world": float("nan"),
            "detected_y_world": float("nan"),
            "detected_z_world": float("nan"),
            "foi_error": float("nan"),
            "success": 0
        }

        if not os.path.exists(self.foi_file):
            rospy.logwarn("FOI file not found: %s", self.foi_file)
            return result

        if not os.path.exists(self.map_origin_file):
            rospy.logwarn("map_origin_file not found: %s", self.map_origin_file)
            return result

        with open(self.foi_file, "r") as f:
            foi_map = json.load(f)

        with open(self.map_origin_file, "r") as f:
            map_origin = json.load(f)

        mx = float(foi_map["x"])
        my = float(foi_map["y"])
        mz = float(foi_map.get("z", 0.0))

        ox = float(map_origin["x"])
        oy = float(map_origin["y"])
        oz = float(map_origin["z"])

        # Here using translation only, assuming map/world axes aligned in your setup.
        # If later needed, replace with full transform.
        wx = ox + mx
        wy = oy + my
        wz = oz + mz

        err = math.sqrt((wx - self.foi_gt_x) ** 2 + (wy - self.foi_gt_y) ** 2 + (wz - self.foi_gt_z) ** 2)

        result["detected_x_world"] = wx
        result["detected_y_world"] = wy
        result["detected_z_world"] = wz
        result["foi_error"] = err
        result["success"] = 1
        return result

    def save_metrics(self, metrics):
        with open(self.detail_json, "w") as f:
            json.dump(metrics, f, indent=2)

        header = [
            "run_id",
            "uav1_rms_position_error_m",
            "uav1_mean_position_error_m",
            "uav1_max_position_error_m",
            "uav1_final_drift_m",
            "map_completeness_percent",
            "foi_detected_x_world_m",
            "foi_detected_y_world_m",
            "foi_detected_z_world_m",
            "foi_gt_x_world_m",
            "foi_gt_y_world_m",
            "foi_gt_z_world_m",
            "foi_position_error_m",
            "foi_detection_success"
        ]

        file_exists = os.path.exists(self.summary_csv)
        with open(self.summary_csv, "a") as f:
            writer = csv.DictWriter(f, fieldnames=header)
            if not file_exists:
                writer.writeheader()
            writer.writerow(metrics)

if __name__ == "__main__":
    MissionMetricsLogger()