#!/usr/bin/env python

# from os import replace
import sys
import rospy
import numpy
from geometry_msgs.msg import Twist
from rospy.core import rospyinfo
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class LaserRangeTesting :
    
    subF = subL = subR = scans = tuple()
    target_acquired = False
    orientation = list()
    target = 0 # degrees to be applied to yaw
    kP = 0.5
    yaw = 0
    
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

    def callbackScans(self, msg) :
        self.subF = msg.ranges

    def callbackOdom(self, odom) :
        orientation = odom.pose.pose.orientation
        self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        # rospy.logerr(yaw) # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        yaw_degrees = (yaw) * math.pi/180
        # rospy.loginfo("Yaw in degress: %s", yaw_degrees)

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

    def quaternions_to_euler_angles(self) :
        if not self.target_acquired :
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            command = Twist()
            target = self.target * math.pi/180 # converting degrees into radians
            # target = (self.target + 90) * math.pi/180 # converting degrees into radians
            if format(target, '.2f') == format(self.yaw, '.2f') :
                command.angular.z = 0
                rospy.logerr('Target Acuired!')
                self.target_acquired = True
            else :
                command.angular.z = self.kP * (target - self.yaw)
                rospy.loginfo('Target: %s, Yaw: %s', target, self.yaw)
                pub.publish(command)

    def scansTest(self) :
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Scans size: %s", len(self.subF))
        rospy.loginfo("-----------------------------------------")

    def printData(self) :
        rospy.loginfo(self.subF)

if __name__ == "__main__" :
    rospy.init_node('laser_scan_values_2')
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    rangeTester = LaserRangeTesting()
    
    # arguments from cmd
    target = False
    if "target=" in str(sys.argv) :
        target_args = sys.argv[1]
        target = target_args.split('=')[1]
        rangeTester.target = int(target)
    count = False
    if "count" in str(sys.argv) :
        count = True
    printData = False
    if "print" in str(sys.argv) :
        printData = True

    rospy.Subscriber('/scan_front', LaserScan, rangeTester.callbackFront)
    rospy.Subscriber('/scan_left', LaserScan, rangeTester.callbackLeft)
    rospy.Subscriber('/scan_right', LaserScan, rangeTester.callbackRight)
    # rospy.Subscriber('/scan_center', LaserScan, rangeTester.callbackScans)
    rospy.Subscriber('/scans', LaserScan, rangeTester.callbackScans)
    rospy.Subscriber('/odom', Odometry, rangeTester.callbackOdom)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # command = Twist()

    while not rospy.is_shutdown():
        # if rangeTester.subR != tuple() and rangeTester.subF != tuple() :
        if rangeTester.subF != tuple() :
            # rangeTester.testing()
            # rangeTester.direction_check()
            # rangeTester.parallel_testing()
            # rangeTester.laser_mid_mesurement()
            if count :
                rangeTester.scansTest()
            if printData :
                rangeTester.printData()
            if target :
                rangeTester.quaternions_to_euler_angles()
        else :
            rospy.loginfo("rangeTester.subR: %s, rangeTester.subFR: %s", rangeTester.subR, rangeTester.subF)

        rate.sleep()