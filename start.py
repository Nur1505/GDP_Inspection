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

rm -f ~/.ros/rtabmap.db ~/.ros/rtabmap.db.back

chmod +x /home/inspection/catkin_ws/src/my_room_world/scripts/lawnmower_pose.py
chmod +x ~/catkin_ws/src/my_room_world/scripts/mapping_supervisor.py

cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
exec roslaunch my_room_world survey_and_map.launch
"""

def main():
    # Run using bash so redirects, ~, source, &&, etc. work.
    result = subprocess.run(["bash", "-lc", SCRIPT])
    raise SystemExit(result.returncode)

if __name__ == "__main__":
    main()
