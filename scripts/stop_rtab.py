#!/usr/bin/env python
import os
import time
import rospy
import subprocess
from std_msgs.msg import Bool

def kill_node(name):
    subprocess.call(["rosnode", "kill", name])

def main():
    rospy.init_node("rtabmap_stop_watcher")

    db_path = rospy.get_param("~db_path", os.path.expanduser("~/.ros/rtabmap.db"))
    export_dir = rospy.get_param("~export_dir", os.path.expanduser("~/catkin_ws/src/my_room_world/rtabmap_rgb_export"))

    rospy.loginfo("Waiting for lawnmower to finish...")
    msg = rospy.wait_for_message("/lawnmower_finished", Bool)

    if not msg.data:
        rospy.logwarn("Received /lawnmower_finished but data=False, not exporting.")
        return

    rospy.loginfo("Lawnmower finished. Stopping RTAB-Map nodes...")
    kill_node("/rtabmap")
    kill_node("/rtabmapviz")
    kill_node("/rgbd_odometry")
    kill_node("/rgbd_sync")

    # Give RTAB-Map a moment to close the database cleanly
    time.sleep(3)

    if not os.path.exists(export_dir):
        os.makedirs(export_dir)

    if not os.path.exists(db_path):
        rospy.logerr("Database not found: %s", db_path)
        return

    rospy.loginfo("Exporting images from DB: %s", db_path)
    ret = subprocess.call([
        "rtabmap-export",
        "--images_id",
        "--poses_camera",
        "--poses_format", "11",
        "--output_dir", export_dir,
        db_path
    ])

    if ret == 0:
        rospy.loginfo("Image export complete. Saved to: %s", export_dir)
    else:
        rospy.logerr("rtabmap-export failed with code %d", ret)

if __name__ == "__main__":
    main()


# #!/usr/bin/env python
# import rospy
# import subprocess
# from std_msgs.msg import Bool

# def main():
#     rospy.init_node("rtabmap_stop_watcher")

#     rospy.loginfo("Waiting for lawnmower to finish...")
#     msg = rospy.wait_for_message("/lawnmower_finished", Bool)

#     if msg.data:
#         rospy.loginfo("Lawnmower finished. Stopping RTAB-Map nodes...")
#         subprocess.call(["rosnode", "kill", "/rtabmap"])
#         subprocess.call(["rosnode", "kill", "/rtabmapviz"])
#         subprocess.call(["rosnode", "kill", "/rgbd_odometry"])
#         subprocess.call(["rosnode", "kill", "/rgbd_sync"])

# if __name__ == "__main__":
#     main()