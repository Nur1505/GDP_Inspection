#!/usr/bin/env python
import os
import json
import math
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from hector_uav_msgs.srv import EnableMotors

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

def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def make_T(x, y, z, qx, qy, qz, qw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = [x, y, z]
    return T

def main():
    rospy.init_node("go_to_foi_uav2")

    map_origin_file = rospy.get_param(
        "~map_origin_file",
        os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export/map_origin.json")
    )
    tag_map_file = rospy.get_param(
        "~tag_map_file",
        os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export/apriltag_median_map.json")
    )

    pose_topic = rospy.get_param("~pose_topic", "/uav2/command/pose")
    enable_srv = rospy.get_param("~enable_motors_srv", "/uav2/enable_motors")
    target_z = rospy.get_param("~target_z", 3.0)
    hover_seconds = rospy.get_param("~hover_seconds", 10.0)

    if not os.path.exists(map_origin_file):
        rospy.logerr("Missing map origin file: %s", map_origin_file)
        return

    if not os.path.exists(tag_map_file):
        rospy.logerr("Missing FOI map result file: %s", tag_map_file)
        return

    with open(map_origin_file, "r") as f:
        origin = json.load(f)

    with open(tag_map_file, "r") as f:
        tag_map = json.load(f)

    T_world_map = make_T(
        origin["x"], origin["y"], origin["z"],
        origin["qx"], origin["qy"], origin["qz"], origin["qw"]
    )

    p_map = np.array([tag_map["x"], tag_map["y"], tag_map["z"], 1.0], dtype=np.float64)
    p_world = np.dot(T_world_map, p_map)

    target_x = float(p_world[0])
    target_y = float(p_world[1])

    rospy.loginfo("FOI in map frame:  (%.3f, %.3f, %.3f)",
                  tag_map["x"], tag_map["y"], tag_map["z"])
    rospy.loginfo("FOI in world frame: (%.3f, %.3f, %.3f)",
                  p_world[0], p_world[1], p_world[2])

    ok = enable_motors(enable_srv, True, timeout=10.0)
    if not ok:
        rospy.logwarn("UAV2 motors not enabled; continuing anyway.")

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    rospy.sleep(1.0)

    msg = PoseStamped()
    command_frame = "uav2/world" #rospy.get_param("~command_frame", "world")
    msg.header.frame_id = command_frame
    msg.pose.position.x = target_x
    msg.pose.position.y = target_y
    msg.pose.position.z = target_z
    msg.pose.orientation = yaw_to_quat(0.0)

    rate = rospy.Rate(10)
    ticks = int(max(1.0, hover_seconds) * 10.0)

    rospy.loginfo("Sending UAV2 to FOI world position...")
    for _ in range(ticks):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Done sending UAV2 goal.")

if __name__ == "__main__":
    main()