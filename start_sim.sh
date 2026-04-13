#!/usr/bin/env bash
set -e

echo "Cleaning old ROS/Gazebo processes..."
rosnode kill -a 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true
pkill -f gazebo_ros_control 2>/dev/null || true
pkill -f controller_manager 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
killall -9 roscore rosmaster 2>/dev/null || true

rm -f ~/.ros/rtabmap.db ~/.ros/rtabmap.db.back


chmod +x ~/catkin_ws/src/inspection_GDP/scripts/pose_to_gazebo.py
chmod +x ~/catkin_ws/src/inspection_GDP/scripts/launch_rtab.py
chmod +x ~/catkin_ws/src/inspection_GDP/scripts/uav2_planner.py
chmod +x ~/catkin_ws/src/inspection_GDP/scripts/lawnmower_pose.py
chmod +x ~/catkin_ws/src/inspection_GDP/scripts/stop_rtab.py
chmod +x ~/catkin_ws/src/inspection_GDP/apriltag_location.py
chmod +x ~/catkin_ws/src/inspection_GDP/scripts/mission_metrics_logger.py

cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash

roslaunch inspection_GDP room.launch