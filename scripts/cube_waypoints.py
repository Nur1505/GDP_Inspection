#!/usr/bin/env python2
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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

    for i, y in enumerate(ys):
        if i % 2 == 0:
            wps.append((x_min, y))
            wps.append((x_max, y))
        else:
            wps.append((x_max, y))
            wps.append((x_min, y))

    return wps


def publish_pose(pub, msg, rate, x, y, z, yaw, ticks):
    for _ in range(ticks):
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = yaw_to_quat(yaw)
        pub.publish(msg)
        rate.sleep()


def main():
    rospy.init_node("cube_waypoints")

    pose_topic = rospy.get_param("~pose_topic", "/command/pose")
    rate_hz = float(rospy.get_param("~rate", 20.0))
    hold_time = float(rospy.get_param("~hold_time", 1.0))
    z = float(rospy.get_param("~z", 3.0))
    speed = float(rospy.get_param("~speed", 0.5))  # meters/sec
    fixed_yaw = float(rospy.get_param("~fixed_yaw", 0.0))  # constant heading

    # Optional staging point before survey starts
    start_x = float(rospy.get_param("~start_x", 0.0))
    start_y = float(rospy.get_param("~start_y", -9.0))

    # Actual spawned model position in Gazebo
    spawn_x = float(rospy.get_param("~spawn_x", start_x))
    spawn_y = float(rospy.get_param("~spawn_y", start_y))
    spawn_z = float(rospy.get_param("~spawn_z", z))

    # Survey area
    x_min = float(rospy.get_param("~survey_x_min", -4.0))
    x_max = float(rospy.get_param("~survey_x_max",  4.0))
    y_min = float(rospy.get_param("~survey_y_min", -4.0))
    y_max = float(rospy.get_param("~survey_y_max",  4.0))
    lane_step = float(rospy.get_param("~lane_step", 1.0))

    survey_wps = make_lawnmower(x_min, x_max, y_min, y_max, lane_step)

    if not survey_wps:
        rospy.logerr("No survey waypoints generated.")
        return

    # Full path: spawn -> staging point -> lawnmower points
    path = [(spawn_x, spawn_y)]
    if (start_x, start_y) != (spawn_x, spawn_y):
        path.append((start_x, start_y))
    path.extend(survey_wps)

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)
    rate = rospy.Rate(rate_hz)

    msg = PoseStamped()
    msg.header.frame_id = "world"

    rospy.sleep(1.0)

    x0, y0 = path[0]
    z0 = spawn_z

    # Hold at initial spawn position briefly
    publish_pose(pub, msg, rate, x0, y0, z0, fixed_yaw, int(2.0 * rate_hz))

    rospy.loginfo("Starting lawnmower pattern with fixed yaw...")

    idx = 0
    while not rospy.is_shutdown():
        if idx >= len(path) - 1:
            rospy.loginfo("Lawnmower complete. Holding final position.")
            publish_pose(pub, msg, rate, x0, y0, z, fixed_yaw, 1)
            continue

        x1, y1 = path[idx + 1]

        dx = x1 - x0
        dy = y1 - y0
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 1e-6:
            idx += 1
            continue

        segment_time = dist / max(speed, 1e-3)
        steps = max(1, int(segment_time * rate_hz))

        for s in range(steps):
            t = float(s + 1) / float(steps)
            t_smooth = t * t * (3.0 - 2.0 * t)

            xi = x0 + dx * t_smooth
            yi = y0 + dy * t_smooth
            zi = z0 + (z - z0) * t_smooth if idx == 0 else z

            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = xi
            msg.pose.position.y = yi
            msg.pose.position.z = zi
            msg.pose.orientation = yaw_to_quat(fixed_yaw)
            pub.publish(msg)
            rate.sleep()

        hold_ticks = max(1, int(hold_time * rate_hz))
        publish_pose(pub, msg, rate, x1, y1, z, fixed_yaw, hold_ticks)

        x0, y0 = x1, y1
        z0 = z
        idx += 1


if __name__ == "__main__":
    main()