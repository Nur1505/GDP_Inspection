#!/usr/bin/env python
import os
import json
import rospy
import subprocess
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

def main():
    rospy.init_node("delayed_rtabmap_launcher")

    save_file = rospy.get_param(
        "~map_origin_file",
        os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export/map_origin.json")
    )

    rospy.loginfo("Waiting for RGBD topics...")
    rospy.wait_for_message("/rgbd/color/image_raw", Image)
    rospy.wait_for_message("/rgbd/depth/image_raw", Image)
    rospy.wait_for_message("/rgbd/color/camera_info", CameraInfo)

    rospy.loginfo("Waiting for lawnmower mission to start...")
    rospy.wait_for_message("/lawnmower_started", Bool)

    rospy.loginfo("Getting initial world pose from /ground_truth/state...")
    gt = rospy.wait_for_message("/ground_truth/state", Odometry)

    data = {
        "x": gt.pose.pose.position.x,
        "y": gt.pose.pose.position.y,
        "z": gt.pose.pose.position.z,
        "qx": gt.pose.pose.orientation.x,
        "qy": gt.pose.pose.orientation.y,
        "qz": gt.pose.pose.orientation.z,
        "qw": gt.pose.pose.orientation.w
    }

    out_dir = os.path.dirname(save_file)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    with open(save_file, "w") as f:
        json.dump(data, f, indent=2)

    rospy.loginfo("Saved map origin to %s", save_file)
    rospy.loginfo("Launching RTAB-Map...")

    subprocess.call([
        "roslaunch",
        "my_room_world",
        "vo_final.launch"
    ])

if __name__ == "__main__":
    main()