#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys

data_left = 0

def sample(data, SIDE):
    global data_left
    # print "--------------" + SIDE + "--------------"
    # print data
    cleanedData = filter(lambda x: not math.isinf(x), data.ranges)
    # print sum(cleanedData)
    # print len(cleanedData)
    # print SIDE + ' Average: ' + str(sum(cleanedData))
    # print len(cleanedData)
    # print sum(cleanedData)/50
    # print "----------------------------"
    data_left = sum(cleanedData)

def check_laser_distance_to_obstacle(SIDE):
    global data_left
    rospy.loginfo('RANGING LEFT : ' + str(data_left))

def setUp():
    rospy.init_node('laser_ranging', anonymous=False)
    rospy.Subscriber('scan_left', LaserScan, sample, 'LEFT')
    # rospy.Subscriber('scan_right', LaserScan, sample, 'RIGHT')
    # rospy.Subscriber("cmd_vel_original", Twist, move)

    argv = rospy.myargv(sys.argv)

    if len(argv) == 2:
        check_laser_distance_to_obstacle(argv[1])


if __name__ == "__main__":
    setUp()
    rospy.spin()
