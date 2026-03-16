#!/usr/bin/env python
import os
import time
import rospy
import subprocess
from std_msgs.msg import Bool

def kill_node(name):
    subprocess.call(["rosnode", "kill", name])

def main():
    rospy.init_node("rtabmap_stop_watcher")

    db_path = rospy.get_param("~db_path", os.path.expanduser("~/.ros/rtabmap.db"))
    export_dir = rospy.get_param(
        "~export_dir",
        os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export")
    )
    apriltag_script = rospy.get_param(
        "~apriltag_script",
        os.path.expanduser("~/catkin_ws/src/my_room_world/apriltag_location.py")
    )
    goto_script = rospy.get_param(
        "~goto_script",
        os.path.expanduser("~/catkin_ws/src/my_room_world/scripts/go_to_FOI.py")
    )

    rospy.loginfo("Waiting for lawnmower to finish...")
    msg = rospy.wait_for_message("/lawnmower_finished", Bool)

    if not msg.data:
        rospy.logwarn("Received /lawnmower_finished but data=False, not exporting.")
        return

    rospy.loginfo("Lawnmower finished. Stopping RTAB-Map nodes...")
    kill_node("/rtabmap")
    kill_node("/rtabmapviz")
    kill_node("/rgbd_odometry")
    kill_node("/rgbd_sync")

    time.sleep(5)

    if not os.path.exists(export_dir):
        os.makedirs(export_dir)

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

    rospy.loginfo("Sending drone to estimated AprilTag world position...")
    ret = subprocess.call(["python3", goto_script], cwd=os.path.dirname(goto_script))
    if ret != 0:
        rospy.logerr("go_to_FOI.py failed with code %d", ret)
        return

    rospy.loginfo("Full post-processing pipeline completed successfully.")

if __name__ == "__main__":
    main()