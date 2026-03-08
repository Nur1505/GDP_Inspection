#!/usr/bin/env python2
import rospy
import tf
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

class PerfectOdomPublisher(object):
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "mapping_cube")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb, queue_size=1)
        rospy.loginfo("perfect_odom: reading /gazebo/model_states for model [%s]", self.model_name)

    def cb(self, msg):
        if self.model_name not in msg.name:
            return

        idx = msg.name.index(self.model_name)
        pose = msg.pose[idx]
        twist = msg.twist[idx]
        now = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose
        odom.twist.twist = twist

        self.odom_pub.publish(odom)

        if self.publish_tf:
            self.br.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                now,
                self.base_frame,
                self.odom_frame
            )

if __name__ == "__main__":
    rospy.init_node("perfect_odom")
    PerfectOdomPublisher()
    rospy.spin()

    # chmod +x ~/catkin_ws/src/my_room_world/scripts/perfect_odom.py