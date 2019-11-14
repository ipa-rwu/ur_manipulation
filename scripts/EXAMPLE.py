#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.init_node('futuras_camera_publisher')
    
    pub_tool_pose = rospy.Publisher('/futuras_camera', PoseStamped,queue_size=10)

    seq=0
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        
        current_pose = geometry_msgs.msg.PoseStamped()
        current_pose.pose.position.x = x1+x2 / 2
        current_pose.pose.position.y = y1+y2 / 2
        current_pose.pose.position.z = 123
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
