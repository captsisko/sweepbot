#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from sensor_msgs.msg import LaserScan

# class LaserRangeTesting :

def callbackLeft(msg) :
    print("Left Array lenght ..."),
    print(len(msg.ranges))
    print('\t0 degrees: '), 
    print(msg.ranges[0])
    print('\t90 degrees: '), 
    print(msg.ranges[25])
    print('\t180 degrees: '), 
    print(msg.ranges[49])

def callbackRight(msg) :
    print("Right Array lenght ..."),
    print(len(msg.ranges))
    print('\t0 degrees: '), 
    print(msg.ranges[0])
    print('\t90 degrees: '), 
    print(msg.ranges[25])
    print('\t180 degrees: '), 
    print(msg.ranges[49])

rospy.init_node('laser_scan_values')
subL = rospy.Subscriber('/scan_left', LaserScan, callbackLeft)
subR = rospy.Subscriber('/scan_right', LaserScan, callbackRight)

rospy.spin()

# if __name__ == "__main__":
#     LaserRangeTesting()