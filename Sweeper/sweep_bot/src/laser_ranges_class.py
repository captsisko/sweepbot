#!/usr/bin/env python

# from os import replace
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rospy.core import loginfo, rospyinfo
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

class LaserRangeTesting :
    
    subF = subL = subR = scans = scan_fore = scan_left = scan_rear = scan_right = tuple()
    regions = {}
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
        self.scans = msg.ranges
        # for index in range(0, 135, 135) :
        # self.scan_fore = range(0, 135, 135)
        # self.regions = {
        #     'starboard_aft' : self.scans[0:135],
        #     'starboard_abeam_aft' : self.scans[136:271],
        #     'starboard_abeam_bow' : self.scans[272:407],
        #     'starboard_bow' : self.scans[408:543],
        #     'port_aft' : self.scans[949:1084],
        #     'port_abeam_aft' : self.scans[814:949],
        #     'port_abeam_bow' : self.scans[679:814],
        #     'port_bow' : self.scans[544:679],
        # }
        self.regions = {
            # 'starboard' : {
            #     'bow'   : {
            #     }
            # },
            # 'test' : {
            #     np.arange(3, 30)
            # },
            'starboard_aft' : self.scans[0:135],
            'starboard_abeam_aft' : self.scans[136:271],
            'starboard_abeam_bow' : self.scans[272:407],
            'starboard_bow' : self.scans[408:543],
            'port_aft' : self.scans[949:1084],
            'port_abeam_aft' : self.scans[814:949],
            'port_abeam_bow' : self.scans[679:814],
            'port_bow' : self.scans[544:679],
        }
        # rospy.loginfo('TESTING: %s', self.regions['test'][7])

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

    # def direction_check(self) :
    #     rospy.loginfo("Right Minimum: %s", self.min(self.subR))
    #     rospy.loginfo("Front Minimum: %s", self.min(self.subF))
    #     rospy.loginfo("Left  Minimum: %s", self.min(self.subL))
    #     rospy.loginfo('-----------------------------')

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

    # def laser_mid_mesurement(self) :
    #     combined_array = self.subFR + self.subR
    #     count = len(combined_array)
    #     mid = len(combined_array)/2
    #     # rospy.loginfo(combined_array)
    #     # rospy.loginfo( "Count: %s, Mid: %s", count, mid )
    #     rospy.loginfo("-----------------------------------------")
    #     rospy.loginfo( "INDEX#0: %s", combined_array[0] )
    #     rospy.loginfo( "INDEX#%s: %s", mid, combined_array[mid] )
    #     rospy.loginfo( "INDEX#73: %s", combined_array[73] )
    #     rospy.loginfo("-----------------------------------------")

    def quaternions_to_euler_angles(self) :
        if not self.target_acquired :
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            command = Twist()
            target = self.target * math.pi/180 # converting degrees into radians
            # target = (self.target + 90) * math.pi/180 # converting degrees into radians
            if format(target, '.1f') == format(self.yaw, '.1f') :
                command.angular.z = 0
                rospy.logerr('Target Aqcuired!')
                self.target_acquired = True
                exit()
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

    def indexTester(self, start, end) :
        rospy.loginfo("Start: %s, End: %s", start, end)
        rospy.loginfo("------------------ Initiating Check -----------------------")
        for index in range(start, end) :
            rospy.loginfo("=> index#%s: %s", index, self.scans[index])
        exit()

    def angles(self, section) :
        if section == 'all' : 
            rospy.loginfo(self.regions)
        else :
            rospy.loginfo(self.regions[section])
        exit()

    def clear_inf_values(self, set) :
        return tuple( item for item in set if item != 26.0 )

    def averaging(self, set) :
        # count = len(set)
        # clean_set = set
        
        clean_set = self.clear_inf_values(set)
        count = len(set)
        
        total = 0
        for i in clean_set :
            total += i
        rospy.loginfo('COUNT ALL DATA: %s', count)
        rospy.loginfo('TOTAL CLEAN DATA: %s', total)
        rospy.loginfo('AVERAGE: %s', total/count)
        rospy.loginfo('To Degrees[cos()]: %s', math.cos(total/count))
        rospy.loginfo('To Degrees[degrees()]: %s', math.degrees(total/count))
        rospy.loginfo('---------------------------------------------------------')

    def distance(self, section) :
        if section == 'bow' :
            rospy.loginfo( len( self.regions['starboard_bow'] ) )
            rospy.loginfo( len( self.clear_inf_values(self.regions['starboard_bow']) ) )
            self.averaging(self.regions['starboard_bow'] + self.regions['port_bow'])
        if section == 'port_bow' :
            count = len(self.regions['port_bow'])
            # rospy.loginfo('COUNT: %s', count)
            # rospy.loginfo('COUNT DIVISION: %s', count/4)
            # sections = self.regions['port_bow'] / 4
            # rospy.loginfo( 'sections[0]: %s', self.regions['port_bow'][0:32] )
            # self.averaging( self.regions['port_bow'][0:32] )
            # rospy.loginfo( 'sections[1]: %s', self.regions['port_bow'][33:66] )
            # self.averaging( self.regions['port_bow'][33:66] )
            # rospy.loginfo( 'sections[2]: %s', self.regions['port_bow'][67:100] )
            # self.averaging( self.regions['port_bow'][67:100] )
            # rospy.loginfo( 'sections[3]: %s', self.regions['port_bow'][101:134] )
            # self.averaging( self.regions['port_bow'][101:134] )
            # self.averaging( self.regions['port_bow'][0:32] + self.regions['port_bow'][33:66] + self.regions['port_bow'][67:100] + self.regions['port_bow'][101:134] )
            data = self.getMedianRange( 3 )
            rospy.loginfo(data)
            self.averaging( data )
            # exit()

        # elif section == 'port' :
        #     return 
        # elif section == 'abeam_starboard' :
        #     return 
        # elif section == 'abeam_port' :
        #     return 
        # rospy.loginfo(self.regions[section])
        else :
            rospy.loginfo('Something went wrong!')
        # exit()

    def getMedianRange(self, range, limits=[33,66]) :
        arr = self.regions['port_bow'][limits[0]:limits[1]]
        arr_length = len(arr)
        
        # arr = np.arange(21)
        # np.random.shuffle(arr)
        # arr_length = len(arr)
        # rospy.loginfo(arr)

        # rospy.loginfo('Length ...: %s', arr_length)
        # if arr_length % 2 > 0 :
        #     rospy.loginfo('ODD length')
        # else :
        #     rospy.loginfo('EVEN length')

        median = arr_length / 2

        rospy.loginfo("RANGE: %s", range)
        rospy.loginfo('Median: %s => value: %s', median, arr[median])
        rospy.loginfo("TYPE: %s", type( int(range) ))

        range = int(range)
        start = stop = (range-1) / 2

        # rospy.loginfo("start: %s, stop: %s", start, stop)

        rospy.loginfo( arr[median-start : median+stop+1] )

        return arr[median-start : median+stop+1]

        exit()

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
    start = 0
    end = 0
    if "range=" in str(sys.argv) :
        range_args = sys.argv[1]
        data = range_args.split('=')[1]
        data_args = data.split(',')
        start = int(data_args[0])
        end = int(data_args[1])

    angles = False
    section = ''
    if "angles=" in str(sys.argv) :
        section_args = sys.argv[1]
        section = section_args.split('=')[1]
        rospy.loginfo('CHECKING : %s', section)
        angles = True

    distance = False
    direction = ''
    if "distance=" in str(sys.argv) :
        direction_args = sys.argv[1]
        direction = direction_args.split('=')[1]
        rospy.loginfo('CHECKING DIRECTION : %s', direction)
        distance = True

    medianValues = False
    midValues = 1
    if "medianValues=" in str(sys.argv) :
        medianValues_args = sys.argv[1]
        midValues = medianValues_args.split('=')[1]
        rospy.loginfo('CHECKING MID-VALUE RANGE : %s', midValues)
        medianValues = True

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
        if rangeTester.scans != tuple() :
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
            # if range :
            #     rangeTester.indexTester(start, end)
            if angles :
                rangeTester.angles(section)
            if distance :
                rangeTester.distance(direction)
            if medianValues :
                rangeTester.getMedianRange(midValues)
        else :
            rospy.loginfo("rangeTester.subR: %s, rangeTester.subFR: %s", rangeTester.subR, rangeTester.subF)

        rate.sleep()