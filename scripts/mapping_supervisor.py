#!/usr/bin/env python2
import os
import rospy
import subprocess
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class MappingSupervisor(object):
    def __init__(self):
        self.z_start = float(rospy.get_param("~z_start", 9.0))  # start mapping after takeoff
        self.odom_topic = rospy.get_param("~odom_topic", "/ground_truth/state")
        self.done_topic = rospy.get_param("~done_topic", "/survey_done")

        # Where to store outputs
        self.db_backup = os.path.expanduser(rospy.get_param("~db_backup", "~/.ros/rtabmap.db.back"))
        self.cloud_out_dir = os.path.expanduser(rospy.get_param("~cloud_out_dir", "~/rtabmap_cloud"))
        self.cloud_topic = rospy.get_param("~cloud_topic", "/rtabmap/cloud_map")

        self.mapping_started = False
        self.survey_done = False

        # RTAB-Map services
        rospy.loginfo("Waiting for RTAB-Map services...")
        rospy.wait_for_service("/rtabmap/reset")
        rospy.wait_for_service("/rtabmap/pause")
        rospy.wait_for_service("/rtabmap/resume")
        rospy.wait_for_service("/rtabmap/backup")

        self.reset_srv  = rospy.ServiceProxy("/rtabmap/reset", Empty)
        self.pause_srv  = rospy.ServiceProxy("/rtabmap/pause", Empty)
        self.resume_srv = rospy.ServiceProxy("/rtabmap/resume", Empty)
        self.backup_srv = rospy.ServiceProxy("/rtabmap/backup", Empty)

        # Start paused, and reset any prior map
        rospy.loginfo("Resetting RTAB-Map and pausing...")
        try:
            self.reset_srv()
        except rospy.ServiceException:
            pass
        self.pause_srv()

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.done_topic, Bool, self.done_cb, queue_size=1)

    def done_cb(self, msg):
        if msg.data and not self.survey_done:
            rospy.loginfo("Survey done signal received.")
            self.survey_done = True
            self.finish()

    def odom_cb(self, msg):
        if self.mapping_started:
            return
        z = msg.pose.pose.position.z
        if z >= self.z_start:
            rospy.loginfo("Altitude %.2f >= %.2f: starting mapping (/rtabmap/resume).", z, self.z_start)
            self.resume_srv()
            self.mapping_started = True

    def finish(self):
        rospy.loginfo("Stopping mapping (/rtabmap/pause)...")
        try:
            self.pause_srv()
        except rospy.ServiceException:
            pass

        rospy.loginfo("Backing up RTAB-Map database (/rtabmap/backup)...")
        try:
            self.backup_srv()
        except rospy.ServiceException:
            pass

        # Save a pointcloud snapshot using pcl_ros pointcloud_to_pcd
        # This writes one or more .pcd files. We'll stop after first.
        out_dir = self.cloud_out_dir
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)

        prefix = os.path.join(out_dir, "room_")
        rospy.loginfo("Saving pointcloud from %s to %s*.pcd ...", self.cloud_topic, prefix)

        # Run pointcloud_to_pcd briefly, then terminate after first write.
        # Note: if /rtabmap/cloud_map is huge, it may take a moment to appear.
        cmd = ["rosrun", "pcl_ros", "pointcloud_to_pcd",
               "input:=%s" % self.cloud_topic,
               "_prefix:=%s" % prefix]
        p = subprocess.Popen(cmd)
        rospy.sleep(3.0)  # enough time to write at least one file in most cases
        p.terminate()

        rospy.loginfo("Done. DB backup should be at ~/.ros/rtabmap.db.back and PCD(s) in %s", out_dir)
        rospy.signal_shutdown("mapping complete")

if __name__ == "__main__":
    rospy.init_node("mapping_supervisor")
    MappingSupervisor()
    rospy.spin()
