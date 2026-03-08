#!/usr/bin/env python2
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def main():
    rospy.init_node("cube_waypoints")

    pose_topic = rospy.get_param("~pose_topic", "/command/pose")
    rate_hz = float(rospy.get_param("~rate", 10.0))
    transition_time = float(rospy.get_param("~transition_time", 8.0))
    hold_time = float(rospy.get_param("~hold_time", 2.0))
    z = float(rospy.get_param("~z", 5.0))

    # Start near the back middle of a 20x20 room
    start_x = float(rospy.get_param("~start_x", 0.0))
    start_y = float(rospy.get_param("~start_y", -9.0))

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    r = rospy.Rate(rate_hz)

    # Simple first test path:
    # back middle -> back right -> back left -> back middle
    waypoints = [
        (start_x, start_y, 0.0),
        (6.0,     start_y, 0.0),
        (-6.0,    start_y, 0.0),
        (start_x, start_y, 0.0),
    ]

    msg = PoseStamped()
    msg.header.frame_id = "world"

    rospy.sleep(1.0)

    # Publish first pose for a short time so the cube starts in the expected place
    x0, y0, yaw0 = waypoints[0]
    for _ in range(int(2.0 * rate_hz)):
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x0
        msg.pose.position.y = y0
        msg.pose.position.z = z
        msg.pose.orientation = yaw_to_quat(yaw0)
        pub.publish(msg)
        r.sleep()

    rospy.loginfo("Starting cube waypoint test...")

    idx = 0
    while not rospy.is_shutdown():
        if idx == len(waypoints) - 1:
            rospy.loginfo("Final waypoint reached, holding position.")
            while not rospy.is_shutdown():
                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = x0
                msg.pose.position.y = y0
                msg.pose.position.z = z
                msg.pose.orientation = yaw_to_quat(yaw0)
                pub.publish(msg)
                r.sleep()

        x1, y1, yaw1 = waypoints[idx + 1]

        steps = max(1, int(transition_time * rate_hz))
        for s in range(steps):
            t = float(s + 1) / float(steps)
            t_smooth = t * t * (3.0 - 2.0 * t)

            xi = x0 + (x1 - x0) * t_smooth
            yi = y0 + (y1 - y0) * t_smooth
            yawi = yaw0 + (yaw1 - yaw0) * t_smooth

            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = xi
            msg.pose.position.y = yi
            msg.pose.position.z = z
            msg.pose.orientation = yaw_to_quat(yawi)
            pub.publish(msg)
            r.sleep()

        hold_ticks = max(1, int(hold_time * rate_hz))
        for _ in range(hold_ticks):
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x1
            msg.pose.position.y = y1
            msg.pose.position.z = z
            msg.pose.orientation = yaw_to_quat(yaw1)
            pub.publish(msg)
            r.sleep()

        idx += 1
        x0, y0, yaw0 = x1, y1, yaw1

if __name__ == "__main__":
    main()


    # chmod +x ~/catkin_ws/src/my_room_world/scripts/cube_waypoints.py