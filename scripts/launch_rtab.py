#!/usr/bin/env python
import os
import json
import rospy
import subprocess
from sensor_msgs.msg import Image, CameraInfo
from std_msgs import msg
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

def main():
    rospy.init_node("delayed_rtabmap_launcher")

    save_file = rospy.get_param(
        "~map_origin_file",
        os.path.expanduser("~/catkin_ws/src/inspection_GDP/rtabmap_rgb_export/map_origin.json")
    )

    rgb_topic = rospy.get_param("~rgb_topic", "/uav1/rgbd/color/image_raw")
    depth_topic = rospy.get_param("~depth_topic", "/uav1/rgbd/depth/image_raw")
    camera_info_topic = rospy.get_param("~camera_info_topic", "/uav1/rgbd/color/camera_info")
    gt_topic = rospy.get_param("~ground_truth_topic", "/uav1/ground_truth/state")

    rospy.loginfo("Waiting for RGBD topics...")
    rospy.wait_for_message(rgb_topic, Image)
    rospy.wait_for_message(depth_topic, Image)
    rospy.wait_for_message(camera_info_topic, CameraInfo)

    rospy.loginfo("Waiting for lawnmower mission to start...")
    rospy.wait_for_message("/lawnmower_started", Bool)

    rospy.loginfo("Getting initial world pose from %s...", gt_topic)
    gt = rospy.wait_for_message(gt_topic, Odometry)

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
        "inspection_GDP",
        "vo_final.launch"
    ])

if __name__ == "__main__":
    main()