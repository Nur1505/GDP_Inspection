#!/usr/bin/env python2
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion

def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def frange(start, stop, step):
    vals = []
    x = start
    while x <= stop + 1e-6:
        vals.append(x)
        x += step
    return vals

def make_lawnmower(x_min, x_max, y_min, y_max, lane_step):
    ys = frange(y_min, y_max, lane_step)
    wps = []

    for y in ys:
        # Go across the lane
        wps.append((x_min, y))
        wps.append((x_max, y))

        # Come back on the same lane
        wps.append((x_min, y))

    return wps

def main():
    rospy.init_node("cube_waypoints")

    pose_topic = rospy.get_param("~pose_topic", "/command/pose")
    rate_hz = float(rospy.get_param("~rate", 10.0))
    transition_time = float(rospy.get_param("~transition_time", 15.0))
    hold_time = float(rospy.get_param("~hold_time", 1.0))
    z = float(rospy.get_param("~z", 2.5))

    start_x = float(rospy.get_param("~start_x", 0.0))
    start_y = float(rospy.get_param("~start_y", -9.0))

    # Aircraft survey area near room center
    x_min = float(rospy.get_param("~survey_x_min", -4.0))
    x_max = float(rospy.get_param("~survey_x_max",  4.0))
    y_min = float(rospy.get_param("~survey_y_min", -4.0))
    y_max = float(rospy.get_param("~survey_y_max",  4.0))
    lane_step = float(rospy.get_param("~lane_step", 1.0))

    survey_wps = make_lawnmower(x_min, x_max, y_min, y_max, lane_step)

    if not survey_wps:
        rospy.logerr("No survey waypoints generated.")
        return

    # Start from back middle, then go to first survey point
    waypoints_pos = [(start_x, start_y), survey_wps[0]]
    waypoints_pos.extend(survey_wps[1:])

    fixed_yaw = 0.0
    waypoints = [(x, y, fixed_yaw) for (x, y) in waypoints_pos]

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    r = rospy.Rate(rate_hz)

    msg = PoseStamped()
    msg.header.frame_id = "world"

    rospy.sleep(1.0)

    x0, y0, yaw0 = waypoints[0]
    for _ in range(int(2.0 * rate_hz)):
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x0
        msg.pose.position.y = y0
        msg.pose.position.z = z
        msg.pose.orientation = yaw_to_quat(yaw0)
        pub.publish(msg)
        r.sleep()

    rospy.loginfo("Starting aircraft survey pattern...")

    idx = 0
    while not rospy.is_shutdown():
        if idx >= len(waypoints) - 1:
            rospy.loginfo("Survey complete. Holding final position.")
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

            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = xi
            msg.pose.position.y = yi
            msg.pose.position.z = z
            msg.pose.orientation = yaw_to_quat(yaw1)
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