#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys

class WallTracker :

    data_left = 0
    cdm_vel_publisher = ''

    def __init__(self):
        rospy.init_node('laser_ranging', anonymous=False)
        self.cdm_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('scan_left', LaserScan, self.sample, 'LEFT')
        # rospy.Subscriber('scan_right', LaserScan, sample, 'RIGHT')
        # rospy.Subscriber("cmd_vel_original", Twist, move)

        # rospy.Subscriber("cmd_vel_original", Twist, self.move)
        rospy.Subscriber("cmd_vel", Twist, self.move)

        rospy.spin()

    def sample(self, data, SIDE):
        cleanedData = filter(lambda x: not math.isinf(x), data.ranges)
        self.data_left = sum(cleanedData)

    def move(self, twist):
        # global data_left
        rospy.loginfo('RANGING LEFT : ' + str(self.data_left))
        rospy.loginfo(twist)
        new_twist = Twist()
        # new_twist = twist
        # rospy.loginfo(new_twist)

        limit = 50

        # Also tried ...
        # new_twist.linear.x = twist.linear.x
        # new_twist.linear.y = twist.linear.y
        # new_twist.linear.z = twist.linear.z
        # new_twist.angular.x = twist.angular.x
        # new_twist.angular.y = twist.angular.y
        # new_twist.angular.z = twist.angular.z

        if(self.data_left > limit):
            new_twist.angular.z = .5
            rospy.loginfo('Data-Left['+str(self.data_left)+'] > ' + str(limit) + '--- Setting to 1')
        else:
            new_twist.angular.z = 0
            rospy.loginfo('Data-Left['+str(self.data_left)+'] < ' + str(limit) + '--- Setting to 0')
        
        self.cdm_vel_publisher.publish(new_twist)


if __name__ == "__main__":
    WallTracker()