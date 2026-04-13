#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf

class PoseToGazebo(object):
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "mapping_cube")
        self.reference_frame = rospy.get_param("~reference_frame", "world")
        self.pose_topic = rospy.get_param("~pose_topic", "/command/pose")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.child_frame = rospy.get_param("~child_frame", "base_link")

        rospy.loginfo("Waiting for /gazebo/set_model_state...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.br = tf.TransformBroadcaster()

        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        rospy.loginfo("pose_to_gazebo listening on %s", self.pose_topic)

    def pose_callback(self, msg):
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = self.reference_frame
        state.pose = msg.pose

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        try:
            self.set_model_state(state)

            if self.publish_tf:
                p = msg.pose.position
                q = msg.pose.orientation
                self.br.sendTransform(
                    (p.x, p.y, p.z),
                    (q.x, q.y, q.z, q.w),
                    rospy.Time.now(),
                    self.child_frame,
                    self.parent_frame
                )

        except rospy.ServiceException as e:
            rospy.logwarn("Failed to set model state: %s", str(e))

if __name__ == "__main__":
    rospy.init_node("pose_to_gazebo")
    PoseToGazebo()
    rospy.spin()