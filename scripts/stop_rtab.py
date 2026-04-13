#!/usr/bin/env python
import os
import time
import json
import rospy
import subprocess
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

def kill_node(name):
    subprocess.call(["rosnode", "kill", name])

def save_occupancy_grid(msg, out_file):
    data = {
        "frame_id": msg.header.frame_id,
        "resolution": msg.info.resolution,
        "width": msg.info.width,
        "height": msg.info.height,
        "origin": {
            "x": msg.info.origin.position.x,
            "y": msg.info.origin.position.y,
            "z": msg.info.origin.position.z,
            "qx": msg.info.origin.orientation.x,
            "qy": msg.info.origin.orientation.y,
            "qz": msg.info.origin.orientation.z,
            "qw": msg.info.origin.orientation.w
        },
        "data": list(msg.data)
    }

    with open(out_file, "w") as f:
        json.dump(data, f)

def main():
    rospy.init_node("rtabmap_stop_watcher")

    db_path = rospy.get_param("~db_path", os.path.expanduser("~/.ros/rtabmap.db"))
    export_dir = rospy.get_param(
        "~export_dir",
        os.path.expanduser("~/catkin_ws/src/inspection_GDP/rtabmap_rgb_export")
    )
    apriltag_script = rospy.get_param(
        "~apriltag_script",
        os.path.expanduser("~/catkin_ws/src/inspection_GDP/apriltag_location.py")
    )
    map_topic = rospy.get_param("~map_topic", "/proj_map")

    rospy.loginfo("Waiting for lawnmower to finish...")
    msg = rospy.wait_for_message("/lawnmower_finished", Bool)

    if not msg.data:
        rospy.logwarn("Received /lawnmower_finished but data=False, not exporting.")
        return

    if not os.path.exists(export_dir):
        os.makedirs(export_dir)

    # --------------------------------------------------
    # Save occupancy map to file
    # --------------------------------------------------
    rospy.loginfo("Waiting for occupancy grid on %s ...", map_topic)
    try:
        occ_msg = rospy.wait_for_message(map_topic, OccupancyGrid, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for occupancy grid on %s", map_topic)
        return

    occ_file = os.path.join(export_dir, "occupancy_map.json")
    save_occupancy_grid(occ_msg, occ_file)
    rospy.loginfo("Saved occupancy grid to %s", occ_file)

    # --------------------------------------------------
    # Stop RTAB-Map nodes
    # --------------------------------------------------
    rospy.loginfo("Lawnmower finished. Stopping RTAB-Map nodes...")
    kill_node("/rtabmap")
    kill_node("/rtabmapviz")
    kill_node("/rgbd_odometry")
    kill_node("/rgbd_sync")

    time.sleep(5)

    if not os.path.exists(db_path):
        rospy.logerr("Database not found: %s", db_path)
        return

    rospy.loginfo("Exporting images + poses from DB: %s", db_path)
    ret = subprocess.call([
        "rtabmap-export",
        "--images_id",
        "--poses_camera",
        "--poses_format", "11",
        "--output_dir", export_dir,
        db_path
    ])

    if ret != 0:
        rospy.logerr("rtabmap-export failed with code %d", ret)
        return

    rospy.loginfo("Export complete. Running AprilTag localization...")
    ret = subprocess.call(["python3", apriltag_script], cwd=os.path.dirname(apriltag_script))
    if ret != 0:
        rospy.logerr("apriltag_location.py failed with code %d", ret)
        return

    rospy.loginfo("Launching UAV2 planner...")

    ret = subprocess.call([
        "rosrun", "inspection_GDP", "uav2_planner.py",
        "_saved_map_file:=" + occ_file,
        "_pose_topic:=/uav2/command/pose",
        "_enable_motors_srv:=/uav2/enable_motors",
        "_world_frame:=uav2/world",
        "_base_frame:=uav2/base_link",
        "_command_frame:=uav2/world",
        "_target_z:=3.0",
        "_hover_before_start:=2.0",
        "_waypoint_reach_time:=2.0",
        "_rate:=10.0",
        "_occupancy_threshold:=50",
        "_inflation_radius_cells:=2",
        "_path_sampling_step:=5",
        "_map_origin_file:=" + os.path.join(export_dir, "map_origin.json"),
        "_tag_map_file:=" + os.path.join(export_dir, "apriltag_median_map.json"),
        "_results_dir:=" + os.path.expanduser("~/simulation_results"),
        "_run_id:=run_%d" % int(time.time()),
        "_foi_gt_x:=2.0",
        "_foi_gt_y:=0.0",
        "_foi_gt_z:=2.1"
    ])

    if ret != 0:
        rospy.logerr("uav2_planner.py failed with code %d", ret)
        return

    rospy.loginfo("Full post-processing + planning pipeline completed successfully.")

if __name__ == "__main__":
    main()