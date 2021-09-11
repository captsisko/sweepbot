#!/usr/bin/env python

# from os import replace
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rospy.core import loginfo, rospyerr, rospyinfo
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import time
from sweep_bot.msg import Space
from sweep_bot.msg import SpaceArray
from simple_pid import PID

class LaserRangeTesting :
    
    subF = subL = subR = scans = scan_fore = scan_left = scan_rear = scan_right = tuple()
    regions = {}
    target_acquired = False
    orientation = list()
    target = 0 # degrees to be applied to yaw
    kP = 0.5
    yaw = 0
    openspace_index = 0
    pid_setpoint = 30
    pid_kp = 4
    pid_ki = 0.5
    pid_kd = 2
    pid_loop = 0
    pub = ''
    
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
            'starboard' : {
                'bow' : {
                    'start' : 0, 
                    'end' : -45,
                    'region' : self.scans[408:543]
                },
                'abeam_bow' : {
                    'start' : -45, 
                    'end' :  -90,
                    'region' : self.scans[272:407]
                },
                'abeam_aft' : {
                    'start' : -90, 
                    'end' : -135,
                    'region' : self.scans[136:271]
                },
                'aft' : {
                    'start' : -135, 
                    'end' : -180,
                    'region' : self.scans[0:135]
                },
            },
            'port'  :   {
                'bow' : self.scans[544:679],
                'abeam_bow' : self.scans[679:814],
                'abeam_aft' : self.scans[814:949],
                'aft' : self.scans[949:1084],
            }
        }

    def callbackOdom(self, odom) :
        orientation = odom.pose.pose.orientation
        self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        # rospy.logerr(yaw) # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        yaw_degrees = (yaw) * math.pi/180
        # rospy.loginfo("Yaw in degress: %s", yaw_degrees)

    def callbackScansFreespace(self, msg) :
        # rospy.loginfo(msg)
        self.freespace = msg

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
            if round(target, 2) == round(self.yaw, 2) :
                command.angular.z = 0
                rospy.logerr('Target Aqcuired!')
                self.target_acquired = True
                exit()
            else :
                command.angular.z = self.kP * (target - self.yaw)
                rospy.loginfo('Target: %s, Yaw: %s', target, self.yaw)
                pub.publish(command)

    def quaternions_to_euler_angles_2(self, point) :
        if not self.target_acquired :
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            command = Twist()
            # target = self.target * math.pi/180 # converting degrees into radians

            target = -3.14159274101 + (point * 0.00579999992624)

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
        # rospy.loginfo( self.scans[135:406] )
        self.target = 0
        self.quaternions_to_euler_angles()
        # if section == 'all' : 
        #     rospy.loginfo(self.regions)
        # else :
        #     sections = section.split('_')
        #     # rospy.loginfo(section)
        #     # sections = {sections}
        #     # sections = sections
        #     rospy.loginfo(sections[2:])
        #     # region = ''
        #     # for index, i in enumerate(sections) :
        #     #     # rospy.logerr("INDEX: %s ==> VALUE: %s", index, i)
        #     #     if index == 0 :
        #     #         region += i
        #     #     else :
        #     #         region += '_'+i
        #     # rospy.loginfo("TESTING : %s", region)
        #     # rospy.loginfo(self.regions[sections[0]])
        #     rospy.loginfo(self.regions['port']['abeam']['aft'])
        # exit()

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
            data = self.getMedianRange( 3 )
            rospy.loginfo(data)
            self.averaging( data )
        else :
            rospy.loginfo('Something went wrong!')

    def getMedianRange(self, range, limits=[33,66]) :
        arr = self.regions['port_bow'][limits[0]:limits[1]]
        arr_length = len(arr)
        median = arr_length / 2

        rospy.loginfo("RANGE: %s", range)
        rospy.loginfo('Median: %s => value: %s', median, arr[median])
        rospy.loginfo("TYPE: %s", type( int(range) ))

        range = int(range)
        start = stop = (range-1) / 2

        rospy.loginfo( arr[median-start : median+stop+1] )

        return arr[median-start : median+stop+1]

        exit()

    def getOpenAir(self, freeSpace) :
        rospy.loginfo('Testing: %s', freeSpace)

        # 'port_aft' : self.scans[949:1084],
        # 'port_abeam_aft' : self.scans[814:949],
        # 'port_abeam_bow' : self.scans[679:814],
        # 'port_bow' : self.scans[544:679],

        port_data = self.regions['port_bow'] + self.regions['port_abeam_bow'] + self.regions['port_abeam_aft'] + self.regions['port_aft']

        self.arr = port_data
        # self.arr = (0.886091411113739, 26.0, 0.8820672631263733, 0.8699256777763367, 0.8702948689460754, 0.8681122064590454, 26.0, 26.0, 26.0, 26.0, 26.0, 0.7634991407394409, 0.7808852791786194, 0.7952740788459778, 26.0, 26.0, 26.0, 26.0, 26.0)

        match_index = self.arr.index(26.0, self.openspace_index)

        space = []
        spaces = []

        for i in range( len(self.arr) ) :
            # rospy.loginfo("Comparing OpenSpace-Index: %s to Array Length: %s", i, len(self.arr)-1)
            # rospy.loginfo("%s ===>>> %s", i, self.arr[i])
            if self.arr[i] == 26.0 :
                match_index = i
                space = []
                for j in range(0, 5) :
                    try :
                        if self.arr[match_index + j] == 26.0 :
                            space.append(True)
                        else :
                            space.append(False)
                        j += 1
                    except IndexError :
                        # rospy.logerr('Out of range')
                        self.openspace_index = 0

                rospy.loginfo(space)

                if space.count(True) == 5 and False not in space :
                    rospy.logwarn('space open-air begining at index: %s', match_index)
                #     rospy.logwarn("Sequence space!")
                    spaces.append(match_index)
                else :
                    rospy.logerr("No sequence space!")

            self.openspace_index += 5

        rospy.loginfo('Spaces ===>>> %s', spaces)

    def frontOfRobot(self) :
        rospy.loginfo( self.regions['port_bow'] )
        exit()

    def msg_test(self) :
        # rospy.loginfo( self.freespace.spaces )

        for freespace in self.freespace.spaces :
            freespace = Space(freespace.region, freespace.spaces)
            # rospy.loginfo( freespace.region )
            spaces = freespace.spaces.split(',')
            rospy.loginfo( spaces[7] )
            rospy.loginfo( int(spaces[7]) )
            # test = freespace.spaces
            # rospy.loginfo(test)

    def laserReadings(self, arg) :
        # 'port_aft' : self.scans[949:1084],
        # 'port_abeam_aft' : self.scans[814:949],
        # 'port_abeam_bow' : self.scans[679:814],
        # 'port_bow' : self.scans[544:679],
        # for i in range(10):
        #     print '%-12i%-12i' % (10 ** i, 20 ** i)
        index = 544
        # for index, value in enumerate(self.regions['port_bow']) :
        for value in self.regions['port_bow'] :
            rospy.loginfo("Index: %s ===>>> Value: %s", index, value)
            index += 1

    def pidTest(self) :
        # data = (50, 40, 36, 15, 25)
        # for index, value in enumerate(data) :
        #     # if index == 0 :
        #     #     prevIndex = 0
        #     # else :
        #     #     prevIndex = data[index-1]
        #     error = data[index] - self.pid_setpoint
        #     rospy.loginfo("Error[%s - %s]: %s",data[index], self.pid_setpoint, error)
        # exit()
        move = Twist()

        rospy.loginfo( self.regions['starboard']['bow']['end'] )
        rospy.loginfo( self.regions['starboard']['abeam_bow']['region'][0] )
        rospy.loginfo( self.regions['starboard']['abeam_aft']['region'][-1] )
        # exit()

        pid = PID(0.3, 0.3, 0.0, 1.5)
        longest_distance_x = self.regions['starboard']['abeam_bow']['region'][0]
        pid_longest_distance_x = round( pid(longest_distance_x), 2 )
        rospy.loginfo("Distance-X: %s, PID: %s", longest_distance_x, pid_longest_distance_x)

        pid = PID(0.1, 0.1, 0.0, 1.5)
        longest_distance_z = self.regions['starboard']['abeam_aft']['region'][-1]
        pid_longest_distance_z = round( pid(longest_distance_z), 2 )
        rospy.loginfo("Distance-Z: %s, PID: %s", longest_distance_z, pid_longest_distance_z)

        if longest_distance_x > 1.5 and longest_distance_z > 1.5 :
            move.linear.x = pid_longest_distance_x * -1
            move.angular.z -= pid_longest_distance_z * -1
            rospy.logwarn("Turning [-]")
            rospy.loginfo(move)

            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            pub.publish( move )
        else :
            rospy.logwarn("Turning [+]")
            # move.angular.z += 0.2
            self.target = 0
            self.quaternions_to_euler_angles()
        rospy.loginfo("-----------------------------------------------------------------------------")

    def getAngle(self) :
        rospy.loginfo( self.regions['starboard']['bow']['end'] )
        x = self.regions['starboard']['abeam_bow']['region'][0]
        y = self.regions['starboard']['abeam_aft']['region'][-1]
        z = 0
        rospy.loginfo( x )
        rospy.loginfo( y )
        # # rospy.loginfo( "TAN [1]: %s", math.tan(x/y) )
        # rospy.loginfo( "TAN Degrees: %s", math.degrees(math.tan(x/y)) )
        # # rospy.loginfo( "TAN [2]: %s", math.tan(y/x) )
        # rospy.loginfo( "TAN Degrees: %s", math.degrees(math.tan(y/x)) )
        # rospy.loginfo("-----------------------------------------------------------------------------")
        
        # # rospy.loginfo( "SIN [1]: %s", math.sin(x/y) )
        # rospy.loginfo( "SIN Degrees: %s", math.degrees(math.sin(x/y)) )
        # # rospy.loginfo( "SIN [2]: %s", math.sin(y/x) )
        # rospy.loginfo( "SIN Degrees: %s", math.degrees(math.sin(y/x)) )
        # rospy.loginfo("-----------------------------------------------------------------------------")
        
        # # rospy.loginfo( "COS [1]: %s", math.cos(x/y) )
        # rospy.loginfo( "COS Degrees: %s", math.degrees(math.cos(x/y)) )
        # # rospy.loginfo( "COS [2]: %s", math.cos(y/x) )
        # rospy.loginfo( "COS Degrees: %s", math.degrees(math.cos(y/x)) )
        # rospy.loginfo("-----------------------------------------------------------------------------")

        z = math.sqrt(x**2 + y**2)
        xd = math.asin(x/z) * 180/math.pi
        yd = math.asin(y/z) * 180/math.pi
        rospy.loginfo(xd)
        rospy.loginfo(yd)
        rospy.loginfo("Rounded xd: %s", round(xd, 1))
        rospy.loginfo("Rounded yd: %s", round(yd, 1))
        rospy.loginfo("-----------------------------------------------------------------------------")

    def parallelize(self) :
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

    getAngle = False
    if "getAngle" in str(sys.argv) :
        getAngle = True

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

    openAirDetected = False
    freeSpace = 1
    if "openAir=" in str(sys.argv) :
        openAir_args = sys.argv[1]
        freeSpace = openAir_args.split('=')[1]
        rospy.loginfo('CHECKING MID-VALUE RANGE : %s', freeSpace)
        openAirDetected = True

    front = False
    if "front" in str(sys.argv) :
        front = True

    msg_test = False
    if "msg_test" in str(sys.argv) :
        msg_test = True

    laserReadingsDetected = False
    freeSpace = 1
    if "laserReadings=" in str(sys.argv) :
        laserReadings_args = sys.argv[1]
        laserReadings = laserReadings_args.split('=')[1]
        # rospy.loginfo('CHECKING MID-VALUE RANGE : %s', laserReadings)
        laserReadingsDetected = True

    pid = False
    if "pid" in str(sys.argv) :
        # pid_args = sys.argv[1]
        # pid = pid_args.split('=')[1]
        # rospy.loginfo('CHECKING MID-VALUE RANGE : %s', laserReadings)
        pid = True

    parallel = False
    if "parallel" in str(sys.argv) :
        parallel = True

    # rospy.Subscriber('/scan_front', LaserScan, rangeTester.callbackFront)
    # rospy.Subscriber('/scan_left', LaserScan, rangeTester.callbackLeft)
    # rospy.Subscriber('/scan_right', LaserScan, rangeTester.callbackRight)
    # rospy.Subscriber('/scan_center', LaserScan, rangeTester.callbackScans)
    rospy.Subscriber('/scans_freespace', SpaceArray, rangeTester.callbackScansFreespace)
    rospy.Subscriber('/scans', LaserScan, rangeTester.callbackScans)
    rospy.Subscriber('/odom', Odometry, rangeTester.callbackOdom)
    rangeTester.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
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
            if openAirDetected :
                rangeTester.getOpenAir(freeSpace)
                # break
            if front :
                rangeTester.frontOfRobot()
                # break
            if msg_test :
                rangeTester.msg_test()
            if laserReadingsDetected :
                rangeTester.laserReadings(laserReadings)
            if pid :
                rangeTester.pidTest()
            if getAngle :
                rangeTester.getAngle()
            if parallel :
                rangeTester.parallelize()
        else :
            # rospy.loginfo("rangeTester.subR: %s, rangeTester.subFR: %s", rangeTester.subR, rangeTester.subF)
            rospy.loginfo("rangeTester.scans: %s", rangeTester.scans)

        rate.sleep()