#!/usr/bin/env python
import rospy
import subprocess
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool

def main():
    rospy.init_node("delayed_rtabmap_launcher")

    rospy.loginfo("Waiting for RGBD topics...")
    rospy.wait_for_message("/rgbd/color/image_raw", Image)
    rospy.wait_for_message("/rgbd/depth/image_raw", Image)
    rospy.wait_for_message("/rgbd/color/camera_info", CameraInfo)

    rospy.loginfo("Waiting for lawnmower mission to start...")
    rospy.wait_for_message("/lawnmower_started", Bool)

    rospy.loginfo("Lawnmower started. Launching RTAB-Map...")
    subprocess.call([
        "roslaunch",
        "my_room_world",
        "vo_final.launch"
    ])

if __name__ == "__main__":
    main()