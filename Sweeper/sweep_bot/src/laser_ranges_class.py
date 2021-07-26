#!/usr/bin/env python

from pickle import REDUCE
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class LaserRangeTesting :
    subFR = -1
    subL = -1
    subR = -1
    def callbackFront(self, msg) :
        # rospy.logerr("Front Laser Data")
        # rospy.loginfo("[Front Laser]Array size :%s ", len(msg.ranges) )
        # rospy.loginfo("[Front Laser]Minimum value :%s ", min(msg.ranges[0:49]) )
        self.subFR = msg.ranges[0:24]
        # rospy.loginfo( self.subFR )
        # rospy.loginfo(msg.ranges[24:49] )

    def callbackLeft(self, msg) :
        # rospy.logerr("Left Laser Data")
        # self.subL = min(msg.ranges[0:49])
        self.subL = msg.ranges[0:49]
        rospy.loginfo("[Left Laser]Array size :%s ", len(self.subL) )
        rospy.loginfo("[Left Laser]Minimum value :%s ", min(self.subL) )

    def callbackRight(self, msg) :
        # rospy.logerr("Right Laser Data")
        self.subR = msg.ranges
        # rospy.loginfo(self.subR)
        # rospy.loginfo("[Right Laser]Array size :%s ", len(self.subR) )
        # rospy.loginfo("[Right Laser]Minimum value :%s ", min(self.subR) )

    def parallel_testing(self) :
        # combined_array = self.subR
        # rospy.loginfo( self.subR )
        # rospy.loginfo( "Length: %s", len(str(self.subR)) )
        # rospy.loginfo( self.subR.count(',') )
        if type(self.subR) == list:
            print("a is a list")
        else:
            print("a is a tuple")
        # rospy.loginfo( self.subR )
        # rospy.loginfo('Length: %s', len(combined_array))

    def testing(self) :
        combined_array = self.subFR + self.subR
        count = len(combined_array)
        total = sum( filter( self.reject_inf, combined_array ) )
        rospy.loginfo('COUNT: %s', count)
        rospy.loginfo('TOTAL: %s', total)
        rospy.logwarn('AVERAGE: %s', total/count)

    def reject_inf(self, entry) :
        if not math.isinf(entry) :
            return True

if __name__ == "__main__" :
    rospy.init_node('laser_scan_values_2')
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    rangeTester = LaserRangeTesting()
    # rangeTester.testing()
    # rangeTester.parallel_testing()

    # rospy.loginfo('testing ... testing')

    subF = rospy.Subscriber('/scan_front', LaserScan, rangeTester.callbackFront)
    # # subL = rospy.Subscriber('/scan_left', LaserScan, rangeTester.callbackLeft)
    subR = rospy.Subscriber('/scan_right', LaserScan, rangeTester.callbackRight)

    while not rospy.is_shutdown():
        if rangeTester.subR != -1 and rangeTester.subFR != -1 :

            rangeTester.testing()
            # rangeTester.parallel_testing()

rate.sleep()