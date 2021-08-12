#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class Angles :
    
    def __init__(self) :
        print("Angles class initiated")

    def drawLine(self) :
        marker = Marker()
        # marker.header.frame_id = 'test'
        # visualization_msgs::Marker marker
        marker.header.frame_id = "chassis"
        marker.header.stamp = rospy.Time()
        marker.ns = "my_namespace"
        marker.id = 0
        # marker.type = visualization_msgs::Marker::SPHERE;
        # marker.action = visualization_msgs::Marker::ADD;
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.pose.position.x = 1
        marker.pose.position.y = 1
        marker.pose.position.z = 1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # vis_pub.publish( marker );
        pub = rospy.Publisher('angle_data', Marker, queue_size=10)
        # pub.publish(marker)

if __name__ == "__main__" :
    rospy.init_node('Angles')
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    angles = Angles()

    while not rospy.is_shutdown():
        rate.sleep()
