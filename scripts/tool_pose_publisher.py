#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('ur_tool_pose_publisher')
    listener = tf.TransformListener()
    pub_tool_pose = rospy.Publisher('/tool_pose', PoseStamped,queue_size=10)

    seq = 0
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/egp_50_tip', '/world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        current_pose = geometry_msgs.msg.PoseStamped()
        current_pose.pose.position.x = trans[0]
        current_pose.pose.position.y = trans[1]
        current_pose.pose.position.z = trans[2]
        current_pose.pose.orientation.x = rot[0]
        current_pose.pose.orientation.y = rot[1]
        current_pose.pose.orientation.z = rot[2]
        current_pose.pose.orientation.w = rot[3]
        current_pose.header.frame_id="/egp_50_tip"
        current_pose.header.stamp = rospy.Time.now()
        current_pose.header.seq = seq
        seq += 1

        pub_tool_pose.publish(current_pose)
        
        rate.sleep()
