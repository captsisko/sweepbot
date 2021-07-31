#!/usr/bin/env python

# from os import replace
import rospy
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class LaserRangeTesting :
    
    subF = subL = subR = tuple()
    
    def callbackFront(self, msg) :
        # rospy.logerr("Front Laser Data")
        # rospy.loginfo("[Front Laser]Array size :%s ", len(msg.ranges) )
        # rospy.loginfo("[Front Laser]Minimum value :%s ", min(msg.ranges[0:49]) )
        # self.subFR = msg.ranges[0:24]
        self.subF = msg.ranges
        # rospy.loginfo( self.subFR )
        # rospy.loginfo(msg.ranges[24:49] )

    def callbackLeft(self, msg) :
        # rospy.logerr("Left Laser Data")
        # self.subL = min(msg.ranges[0:49])
        self.subL = msg.ranges
        # rospy.loginfo("[Left Laser]Array size :%s ", len(self.subL) )
        # rospy.loginfo("[Left Laser]Minimum value :%s ", min(self.subL) )

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

    def direction_check(self) :
        rospy.loginfo("Right Minimum: %s", self.min(self.subR))
        rospy.loginfo("Front Minimum: %s", self.min(self.subF))
        rospy.loginfo("Left  Minimum: %s", self.min(self.subL))
        rospy.loginfo('-----------------------------')

    def testing(self) :
        self.subR = self.replace_inf(self.subR)
        rospy.loginfo('-----------------------------')
        rospy.loginfo("Minimum: %s", min(self.subR))
        rospy.loginfo("Minimum(c): %s", self.min(self.subR))
        rospy.loginfo("Maximum: %s", max(self.subR))
        rospy.loginfo("Mean: %s", numpy.mean(self.subR))
        rospy.loginfo("Mean Test: %s", numpy.mean(self.subR))

    def min(self, iTuple) :
        iList = list(iTuple)
        smallest = 1000000000
        for index in range(len(iList)) :
            if iList[index] != 0 :
                smallest = min( smallest, iList[index] )
        # rospy.loginfo(iList)
        return smallest

    def replace_inf(self, iTuple) :
        iList = list(iTuple)
        for index in range(len(iList)) :
            if math.isinf( iList[index] ) :
                iList.pop(index)
        rospy.loginfo(iList)
        return tuple(iList)

    def reject_inf(self, entry) :
        if not math.isinf(entry) :
            return True

    def laser_mid_mesurement(self) :
        combined_array = self.subFR + self.subR
        count = len(combined_array)
        mid = len(combined_array)/2
        # rospy.loginfo(combined_array)
        # rospy.loginfo( "Count: %s, Mid: %s", count, mid )
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo( "INDEX#0: %s", combined_array[0] )
        rospy.loginfo( "INDEX#%s: %s", mid, combined_array[mid] )
        rospy.loginfo( "INDEX#73: %s", combined_array[73] )
        rospy.loginfo("-----------------------------------------")

if __name__ == "__main__" :
    rospy.init_node('laser_scan_values_2')
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    rangeTester = LaserRangeTesting()
    # rangeTester.testing()
    # rangeTester.parallel_testing()

    # rospy.loginfo('testing ... testing')

    subF = rospy.Subscriber('/scan_front', LaserScan, rangeTester.callbackFront)
    subL = rospy.Subscriber('/scan_left', LaserScan, rangeTester.callbackLeft)
    subR = rospy.Subscriber('/scan_right', LaserScan, rangeTester.callbackRight)

    while not rospy.is_shutdown():
        if rangeTester.subR != tuple() and rangeTester.subF != tuple() :
            # rangeTester.testing()
            rangeTester.direction_check()
            # rangeTester.parallel_testing()
            # rangeTester.laser_mid_mesurement()
        else :
            rospy.loginfo("rangeTester.subR: %s, rangeTester.subFR: %s", rangeTester.subR, rangeTester.subF)

        rate.sleep()