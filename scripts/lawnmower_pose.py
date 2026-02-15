#!/usr/bin/env python2
import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from hector_uav_msgs.srv import EnableMotors 

def enable_motors(service_name="/enable_motors", enable=True, timeout=10.0):
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

def yaw_to_quat(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def make_waypoints(room_size=6.0, margin=2.0, lane_step=0.6):
    half = room_size / 2.0
    x_min = -half + margin
    x_max =  half - margin
    y_min = -half + margin
    y_max =  half - margin

    ys = []
    y = y_min
    while y <= y_max + 1e-6:
        ys.append(y)
        y += lane_step

    wps = []
    direction = 1
    for y in ys:    
        if direction == 1:
            wps.append((x_min, y))
            wps.append((x_max, y))
        else:
            wps.append((x_max, y))
            wps.append((x_min, y))
        direction *= -1
    return wps

def main():
    rospy.init_node("lawnmower_pose")

    pose_topic = rospy.get_param("~pose_topic", "/command/pose")
    enable_srv = rospy.get_param("~enable_motors_srv", "/enable_motors")

    #ENABLING MOTORS
    ok = enable_motors(enable_srv, True, timeout=10.0)
    if not ok:
        rospy.logwarn("Motors not enabled; continuing anyway.")
    # ---------------------------------------------

    room_size = float(rospy.get_param("~room_size", 30.0))
    margin = float(rospy.get_param("~margin", 2.0))
    lane_step = float(rospy.get_param("~lane_step", 1.5))
    z = float(rospy.get_param("~z", 10.0))

    rate_hz = float(rospy.get_param("~rate", 10.0))
    hold_time = float(rospy.get_param("~hold_time", 0.0))
    transition_time = float(rospy.get_param("~transition_time", 10.0))
    yaw_transition_time = float(rospy.get_param("~yaw_transition_time", 1.0))

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)

    waypoints_pos = make_waypoints(room_size=room_size, margin=margin, lane_step=lane_step)
    # compute yaw aligned with direction of motion (angle to next waypoint)
    waypoints = []
    for i, (x, y) in enumerate(waypoints_pos):
        nx, ny = waypoints_pos[(i + 1) % len(waypoints_pos)]
        yaw = math.atan2(ny - y, nx - x)
        waypoints.append((x, y, yaw))
    rospy.loginfo("Generated %d waypoint points (serpentine, yaw aligned).", len(waypoints))

    r = rospy.Rate(rate_hz)

    msg = PoseStamped()
    msg.header.frame_id = "world"

    ticks_per_wp = max(1, int(hold_time * rate_hz))

    # determine start index: if start_x/start_y provided, pick nearest waypoint
    idx = 0
    start_x = rospy.get_param("~start_x", None)
    start_y = rospy.get_param("~start_y", None)
    if start_x is not None and start_y is not None:
        best = None
        best_idx = 0
        for i, (wx, wy, wyaw) in enumerate(waypoints):
            d2 = (wx - float(start_x))**2 + (wy - float(start_y))**2
            if best is None or d2 < best:
                best = d2
                best_idx = i
        idx = best_idx
        rospy.loginfo("Starting at nearest waypoint %d to start=(%.2f, %.2f)", idx, float(start_x), float(start_y))

    # publish initial pose
    x0, y0, yaw0 = waypoints[idx]
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = x0
    msg.pose.position.y = y0
    msg.pose.position.z = 10
    msg.pose.orientation = yaw_to_quat(yaw0)
    pub.publish(msg)

    while not rospy.is_shutdown():
        next_idx = (idx + 1) % len(waypoints)
        x1, y1, yaw1 = waypoints[next_idx]

        # transition (interpolate) from current waypoint to next
        if transition_time and transition_time > 0.0:
            pos_steps = max(1, int(transition_time * rate_hz))
            yaw_steps = max(1, int(yaw_transition_time * rate_hz))
            steps = max(pos_steps, yaw_steps)
            for s in range(steps):
                # position interpolation (can finish before yaw)
                t_pos = min(1.0, float(s + 1) / float(pos_steps))
                xi = x0 + (x1 - x0) * t_pos
                yi = y0 + (y1 - y0) * t_pos
                zi = 10
                # yaw interpolation (independent timing)
                t_yaw = min(1.0, float(s + 1) / float(yaw_steps))
                dy = (yaw1 - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
                yawi = yaw0 + dy * t_yaw

                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = xi
                msg.pose.position.y = yi
                msg.pose.position.z = zi
                msg.pose.orientation = yaw_to_quat(yawi)
                pub.publish(msg)
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    return

        else:
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x1
            msg.pose.position.y = y1
            msg.pose.position.z = 10
            msg.pose.orientation = yaw_to_quat(yaw1)
            pub.publish(msg)

        # hold at the waypoint for hold_time seconds
        for _ in range(ticks_per_wp):
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x1
            msg.pose.position.y = y1
            msg.pose.position.z = 10
            msg.pose.orientation = yaw_to_quat(yaw1)
            pub.publish(msg)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                return


        # advance
        idx = next_idx
        x0, y0, yaw0 = x1, y1, yaw1

if __name__ == "__main__":
    main()

