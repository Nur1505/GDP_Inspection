#!/usr/bin/env python3
import subprocess
import sys

SCRIPT = r"""
set -e

rosnode kill -a 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true
pkill -f gazebo_ros_control 2>/dev/null || true
pkill -f controller_manager 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
killall -9 roscore rosmaster 2>/dev/null || true

 export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/catkin_ws/src/my_room_world

rm -f ~/.ros/rtabmap.db ~/.ros/rtabmap.db.back

chmod +x /home/inspection/catkin_ws/src/my_room_world/scripts/lawnmower_pose.py
chmod +x ~/catkin_ws/src/my_room_world/scripts/mapping_supervisor.py
chmod +x ~/catkin_ws/src/my_room_world/scripts/pose_to_gazebo.py
chmod +x ~/catkin_ws/src/my_room_world/scripts/perfect_odom.py
chmod +x ~/catkin_ws/src/my_room_world/scripts/cube_waypoints.py

cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
roslaunch my_room_world room.launch
roslaunch my_room_world survey_and_map.launch

roslaunch my_room_world rtabmap_perfect_odom.launch

"""

def main():
    # Run using bash so redirects, ~, source, &&, etc. work.
    result = subprocess.run(["bash", "-lc", SCRIPT])
    raise SystemExit(result.returncode)

if __name__ == "__main__":
    main()

# namjestit yaw da ne bude tu nego samo da se krece onako u l obliku
# provjerit onaj depth koji sam dodatno instalirala sto nemam depth_image_proc nakon instalacije ros-melodic-depth-image-proc



# <!-- inspection@inspection-ThinkStation-P320:~/catkin_ws$ roscore
# ... logging to /home/inspection/.ros/log/91ee937c-1260-11f1-aab7-309c23723405/roslaunch-inspection-ThinkStation-P320-30212.log
# Checking log directory for disk usage. This may take a while.
# Press Ctrl-C to interrupt
# Done checking log file disk usage. Usage is <1GB.

# started roslaunch server http://inspection-ThinkStation-P320:41449/
# ros_comm version 1.14.13


# SUMMARY
# ========

# PARAMETERS
#  * /rosdistro: melodic
#  * /rosversion: 1.14.13

# NODES

# RLException: roscore cannot run as another roscore/master is already running. 
# Please kill other roscore/master processes before relaunching.
# The ROS_MASTER_URI is http://inspection-ThinkStation-P320:11311/
# The traceback for the exception was written to the log file
# inspection@inspection-ThinkStation-P320:~/catkin_ws$ ps aux | grep roscore
# inspect+ 30236  0.0  0.0  14432  1008 pts/0    S+   15:46   0:00 grep --color=auto roscore
# inspection@inspection-ThinkStation-P320:~/catkin_ws$ sudo lsof -i :11311
# [sudo] password for inspection: 
# Sorry, try again.
# [sudo] password for inspection: 
# COMMAND     PID       USER   FD   TYPE DEVICE SIZE/OFF NODE NAME -->
