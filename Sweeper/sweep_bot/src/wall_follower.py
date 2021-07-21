#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from sensor_msgs.msg import LaserScan

class Tasker :
    
    subL_value = ''
    subR_value = 0
    # def __init__(self) :

    def callbackLeft(self, msg) :
        subL_value = min(msg.ranges[0:49])
        self.set('Left', subL_value)
        # rospy.loginfo(subL_value)

    def callbackRight(self, msg) :
        subR_value = min(msg.ranges[0:49])
        self.set('Right', subR_value)
        # rospy.loginfo(subR_value)

    def set(self, value1, value2) :
        # rospy.loginfo("%s : %s", value1, value2)
        # test = math.inf
        test = float("inf") + 0
        rospy.loginfo("INF : %s", test),
        # rospy.loginfo(value2)

    # def run(self) :
        # rospy.loginfo('\tLeft: '), 
        # rospy.loginfo(subL_value)
        # rospy.loginfo('Right: '), 
        # rospy.loginfo(self.subR_value)

if __name__ == "__main__":
    rospy.init_node('wall_detector')
    tasker = Tasker()
    rospy.Subscriber('/scan_left', LaserScan, tasker.callbackLeft)
    rospy.Subscriber('/scan_right', LaserScan, tasker.callbackRight)
    rate = rospy.Rate(10) # Fixed update frequency of 10hz
    while not rospy.is_shutdown():
        # rospy.Subscriber('/scan_right', LaserScan, tasker.callbackRight)
        # tasker.run()
        rate.sleep() 
    # rospy.spin()