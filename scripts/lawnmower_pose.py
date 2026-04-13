#!/usr/bin/env python2
import math
from time import time
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from hector_uav_msgs.srv import EnableMotors
from std_msgs.msg import Bool

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

def make_waypoints(room_size_x=30.0, room_size_y=20.0, margin=2.0, lane_step=1.0):
    half_x = room_size_x / 2.0
    half_y = room_size_y / 2.0

    x_min = -half_x + margin
    x_max =  half_x - margin
    y_min = -half_y + margin
    y_max =  half_y - margin

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

    delay = float(rospy.get_param("~start_delay", 0.0))
    if delay > 0:
        rospy.loginfo("Delaying start by %.1f seconds...", delay)
        rospy.sleep(delay)

    # Wait for controller manager
    switch_srv = rospy.get_param("~switch_controller_srv", "/controller_manager/switch_controller")
    unload_srv = rospy.get_param("~unload_controller_srv", "/controller_manager/unload_controller")

    rospy.loginfo("Waiting for controller manager services...")
    rospy.wait_for_service(switch_srv)
    rospy.wait_for_service(unload_srv)

    pose_topic = rospy.get_param("~pose_topic", "/command/pose")
    enable_srv = rospy.get_param("~enable_motors_srv", "/enable_motors")

    # Enable motors
    ok = enable_motors(enable_srv, True, timeout=10.0)
    if not ok:
        rospy.logwarn("Motors not enabled; continuing anyway.")

    # Parameters
    room_size_x = float(rospy.get_param("~room_size_x", 50.0))
    room_size_y = float(rospy.get_param("~room_size_y", 60.0))
    margin      = float(rospy.get_param("~margin", 2.0))
    lane_step   = float(rospy.get_param("~lane_step", 1.0))
    z           = float(rospy.get_param("~z", 5.0))

    rate_hz         = float(rospy.get_param("~rate", 10.0))
    hold_time       = float(rospy.get_param("~hold_time", 0.0))
    transition_time = float(rospy.get_param("~transition_time", 10.0))

    pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)

    waypoints_pos = make_waypoints(room_size_x, room_size_y, margin, lane_step)

    fixed_yaw = 0.0
    waypoints = [(x, y, fixed_yaw) for (x, y) in waypoints_pos]

    r = rospy.Rate(rate_hz)

    command_frame = rospy.get_param("~command_frame", "world")

    msg = PoseStamped()
    msg.header.frame_id = command_frame

    ticks_per_wp = max(1, int(hold_time * rate_hz))

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

    # -------------------------------
    # 1. Vertical takeoff to target altitude
    # -------------------------------
    x0, y0, yaw0 = waypoints[idx]

    rospy.loginfo("Taking off vertically to z=%.2f..." % z)

    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = x0
    msg.pose.position.y = y0
    msg.pose.position.z = z
    msg.pose.orientation = yaw_to_quat(fixed_yaw)

    # Publish repeatedly until altitude is reached
    takeoff_rate = rospy.Rate(10)
    for _ in range(50):  #5 seconds of vertical climb commands
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        takeoff_rate.sleep()

    # -------------------------------
    # 2. Hover for a moment before starting lawnmower pattern
    # -------------------------------
    rospy.loginfo("Hovering...")
    hover_rate = rospy.Rate(10)
    for _ in range(60):
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        hover_rate.sleep()

    rospy.loginfo("Starting lawnmower pattern...")

    started_pub = rospy.Publisher("/lawnmower_started", Bool, queue_size=1, latch=True)
    finished_pub = rospy.Publisher("/lawnmower_finished", Bool, queue_size=1, latch=True)
    
    started_pub.publish(Bool(data=True))
    
    hover_rate = rospy.Rate(10)
    for _ in range(60):
        hover_rate.sleep()
        
    while not rospy.is_shutdown():
        if idx == len(waypoints) - 1: 
            rospy.loginfo("Reached final waypoint. Holding position and stopping mission.") 
            # Hold final pose for 3 seconds 
            hold_rate = rospy.Rate(10) 
            for _ in range(30): 
                msg.header.stamp = rospy.Time.now() 
                msg.pose.position.x = x0 
                msg.pose.position.y = y0 
                msg.pose.position.z = z 
                msg.pose.orientation = yaw_to_quat(fixed_yaw) 
                pub.publish(msg) 
                hold_rate.sleep()
            finished_pub.publish(Bool(data=True)) 
            rospy.loginfo("Published lawnmower finished signal.")
            rospy.sleep(0.5)
            return 
    
        next_idx = (idx + 1) % len(waypoints)
        x1, y1, yaw1 = waypoints[next_idx]

        # Smooth transition
        if transition_time > 0.0:
            steps = max(1, int(transition_time * rate_hz))
            for s in range(steps):
                t = float(s + 1) / float(steps)
                # xi = x0 + (x1 - x0) * t
                # yi = y0 + (y1 - y0) * t
                t_smooth = t * t * (3 - 2 * t)
                xi = x0 + (x1 - x0) * t_smooth  
                yi = y0 + (y1 - y0) * t_smooth

                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = xi
                msg.pose.position.y = yi
                msg.pose.position.z = z
                msg.pose.orientation = yaw_to_quat(fixed_yaw)
                pub.publish(msg)
                r.sleep()
        else:
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x1
            msg.pose.position.y = y1
            msg.pose.position.z = z
            msg.pose.orientation = yaw_to_quat(fixed_yaw)
            pub.publish(msg)

        idx = next_idx
        x0, y0, yaw0 = x1, y1, yaw1

if __name__ == "__main__":
    main()
