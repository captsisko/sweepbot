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
from sweep_bot.msg import Angle
from sweep_bot.msg import AngleArray
from sweep_bot.msg import AnglesArray
import settings as CONFIG

class LaserRangeTesting :
    # RANGE = 6.0
    
    subF = subL = subR = scans = scan_fore = scan_left = scan_rear = scan_right = tuple()
    regions = regions_angles = {}
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
    previous_distance_to_wall = 0
    previous_distance_set = False
    
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
        self.regions = {
            'starboard_aft' : self.scans[0:135],
            'starboard_abeam_aft' : self.scans[136:271],
            'starboard_abeam_bow' : self.scans[272:407],
            'starboard_bow' : self.scans[408:543],
            'port_aft' : self.scans[949:1084],
            'port_abeam_aft' : self.scans[814:949],
            'port_abeam_bow' : self.scans[679:814],
            'port_bow' : self.scans[544:679],
            'starboard_abeam' : self.scans[136:407],
            'port_abeam' : self.scans[679:949],
            'bow' : self.scans[408:679],
        }
        # self.regions = {
        #     'starboard' : {
        #         'bow' : self.scans[408:543],
        #         'abeam_bow' : self.scans[272:407],
        #         'abeam_aft' : self.scans[136:271],
        #         'aft' : self.scans[0:135],
        #     },
        #     'port'  :   {
        #         'bow' : self.scans[544:679],
        #         'abeam_bow' : self.scans[679:814],
        #         'abeam_aft' : self.scans[814:949],
        #         'aft' : self.scans[949:1084],
        #     }
        # }

    def callbackOdom(self, odom) :
        orientation = odom.pose.pose.orientation
        self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        # rospy.logerr(yaw) # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        # yaw_degrees = (yaw) * math.pi/180
        yaw_degrees = (yaw) * 180/math.pi
        # rospy.loginfo("Yaw => Radians : [%s], Degress : [%s]", yaw, (yaw_degrees+90)*1)
        # rospy.loginfo("Yaw => Degress : [%s]", yaw_degrees+90)

    def callbckRegionsAngle(self, anglesArray) :
        for regionAngles in anglesArray.angles :
            self.regions_angles[regionAngles.region] = {
                'angles' : regionAngles.angles,
                'parallel' : regionAngles.parallel,
                'mid_distance' : regionAngles.mid_distance,
                'avg_distance' : regionAngles.avg_distance
            }
            # rospy.loginfo("processing: %s",regionAngles.region)
        # rospy.loginfo(self.regions_angles)


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

    def printData(self, section='all') :
        rospy.loginfo('Printing section: %s', section)
        if section == 'all' :
            rospy.loginfo( self.regions )
        else :
            rospy.loginfo( self.regions[section] )
        exit()

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
        return tuple( item for item in set if item != CONFIG.RANGE )

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
        # self.arr = (0.886091411113739, CONFIG.RANGE, 0.8820672631263733, 0.8699256777763367, 0.8702948689460754, 0.8681122064590454, CONFIG.RANGE, CONFIG.RANGE, CONFIG.RANGE, CONFIG.RANGE, CONFIG.RANGE, 0.7634991407394409, 0.7808852791786194, 0.7952740788459778, CONFIG.RANGE, CONFIG.RANGE, CONFIG.RANGE, CONFIG.RANGE, CONFIG.RANGE)

        match_index = self.arr.index(CONFIG.RANGE, self.openspace_index)

        space = []
        spaces = []

        for i in range( len(self.arr) ) :
            # rospy.loginfo("Comparing OpenSpace-Index: %s to Array Length: %s", i, len(self.arr)-1)
            # rospy.loginfo("%s ===>>> %s", i, self.arr[i])
            if self.arr[i] == CONFIG.RANGE :
                match_index = i
                space = []
                for j in range(0, 5) :
                    try :
                        if self.arr[match_index + j] == CONFIG.RANGE :
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

    def findrightwall(self) :
        move = Twist()

        if self.regions['starboard_bow'][0] <= 1.5 :
            rospy.loginfo("Found right wall")
            move.linear.x = 0.0
            move.angular.z -= 0.0
            exit()

        else :
            rospy.loginfo("Finding right wall :")
            move.linear.x = 0.3
            move.angular.z -= 0.2

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub.publish( move )

    def parallel_park(self) :

        if not self.regions_angles['starboard_abeam']['parallel'] :
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            move = Twist()
            rospy.logwarn("Driving to parallel . . .")
            move.angular.z = 0.2
            pub.publish( move )
        else :
            rospy.logerr("Now parallel to wall")
            # move.linear.x = 0.0
            # move.angular.z = 0.0
            # exit()


    # def pidRight(self) :
    #     rospy.loginfo("----------------- PROCESSING PID-RIGHT : starboard_abeam_bow= %s", self.regions['starboard_abeam_bow'][0])

    #     move = Twist()

    #     rospy.logerr("TESTING: %s", CONFIG.RANGE)

    #     if self.regions['starboard_abeam_bow'][0] == CONFIG.RANGE :
    #         pid = PID(0.15, 0.3, 0.0, 1)
    #         longest_distance_x = self.regions['starboard_abeam_bow'][0]
    #         pid_longest_distance_x = round( pid(longest_distance_x), 2 )
    #         rospy.loginfo("Distance-X: %s, PID: %s", longest_distance_x, pid_longest_distance_x)

    #         pid = PID(0.07, 0.1, 0.0, 1)
    #         longest_distance_z = self.regions['starboard_abeam_aft'][-1]
    #         pid_longest_distance_z = round( pid(longest_distance_z), 2 )
    #         rospy.loginfo("Distance-Z: %s, PID: %s", longest_distance_z, pid_longest_distance_z)
    #     else :
    #         pid = PID(0.15, 0.3, 0.0, 0)
    #         longest_distance_x = self.regions['starboard_abeam_bow'][0]
    #         pid_longest_distance_x = round( pid(longest_distance_x), 2 )
    #         rospy.loginfo("Distance-X: %s, PID: %s", longest_distance_x, pid_longest_distance_x)

    #         pid = PID(0.007, 0.1, 0.0, 0)
    #         longest_distance_z = self.regions['starboard_abeam_aft'][-1]
    #         pid_longest_distance_z = round( pid(longest_distance_z), 2 )
    #         rospy.loginfo("Distance-Z: %s, PID: %s", longest_distance_z, pid_longest_distance_z)

    #     if longest_distance_x > 1.5 and longest_distance_z > 1.5 :
    #         move.linear.x = pid_longest_distance_x * -1
    #         move.angular.z -= pid_longest_distance_z * -1
    #         rospy.logwarn("Turning [-]")
    #         rospy.loginfo(move)

    #         pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #         pub.publish( move )
    #     else :
    #         rospy.logwarn("Stopping ..... ")
    #         move.linear.x = 0
    #         move.angular.z -= 0

    #         pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #         pub.publish( move )

    #         exit()

    def pidRight(self) :
        # rospy.loginfo("----------------- PROCESSING PID-RIGHT : starboard_abeam_bow= %s", self.regions_angles['starboard']['avg_distance'])

        move = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        distance_to_wall = self.regions_angles['starboard']['avg_distance']
        pid = PID(0.5, 0.0, 0.0, 0.5)
        pid_distance_to_wall = pid(distance_to_wall)

        move.linear.x = 0.5

        if distance_to_wall < 1.0 :
            move.angular.z -= pid_distance_to_wall
            rospy.logerr("%s >>>>>", distance_to_wall)

        if self.regions_angles['bow']['mid_distance'] < 1.0 :
            # move.angular.z += pid_distance_to_wall
            # rospy.logwarn("%s <<<<<", distance_to_wall)
            # move.linear.x -= 0.5
            self.parallel_park()
            exit()

        rospy.loginfo(move)
        pub.publish( move )

    def pidTurn(self) :

        rospy.loginfo("----------------- PROCESSING PID-TURN -----------------")
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        move = Twist()

        rospy.loginfo("TESTING: %s", self.regions_angles['bow']['parallel'])
        if not self.regions_angles['bow']['parallel'] :
            move.angular.z -= 0.2
        else :
            move.angular.z = 0
            rospy.logerr("STOP!!!")

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub.publish( move )


    def pidLine(self) :
        # rospy.loginfo("----------------- PROCESSING PID-LINE : %s", self.regions_angles['starboard']['avg_distance'])

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        move = Twist()
        distance_to_wall = self.regions_angles['starboard_abeam']['mid_distance']
        set_point = 1.0
        pid = PID(0.1, 0.0, 0.0, set_point)
        pid_distance_to_wall = pid( distance_to_wall )

        if not self.previous_distance_set :
            self.previous_distance_to_wall = distance_to_wall
            self.previous_distance_set = True

        move.linear.x = 0.5

        # rospy.loginfo("PID: %s, PID-Distance: %s", pid, pid_distance_to_wall)
        # pid.tunings = (1.0, 0.2, 0.4)
        # pid.setpoint = 0.5

        # rospy.loginfo("Value#1: %s, Value#2: %s", self.regions_angles['starboard_abeam']['angles'][0], self.regions_angles['starboard_abeam']['angles'][1] )
        if not self.regions_angles['starboard_abeam']['parallel'] :
            rospy.logwarn_once('Re-orienting!')        
            pid = PID(0.1, 0.0, 0.0, 45.0)
            pid_parallel_to_wall = pid( self.regions_angles['starboard_abeam']['angles'][0] )
            rospy.loginfo("PID-Distance: %s", pid_parallel_to_wall)

            move.linear.x = 0
            move.angular.z = 1.0
        else :
            # if not self.regions_angles['starboard_abeam']['parallel'] :
            rospy.logerr("Should stop here ...")
            # else :
            move.angular.z = 0
            #     rospy.logerr("Test#2")

        # else :

        #     if self.isclose(self.previous_distance_to_wall, distance_to_wall, 0.1) :
        #         if distance_to_wall > 1.0 :
        #             rospy.logwarn("%s >>>>>", distance_to_wall)
        #             move.angular.z -= pid_distance_to_wall * -1
        #         else :
        #             rospy.logerr("%s <<<<<", distance_to_wall)
        #             move.angular.z += pid_distance_to_wall

        #         self.previous_distance_to_wall = distance_to_wall

        pub.publish( move )

    def isclose(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def getAngle(self) :
        # rospy.loginfo( self.regions['starboard']['bow']['end'] )
        # x = self.regions['starboard']['abeam_bow']['region'][0]
        # y = self.regions['starboard']['abeam_aft']['region'][-1]
        # z = 0
        # rospy.loginfo( x )
        # rospy.loginfo( y )
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

        # z = math.sqrt(x**2 + y**2)
        # xd = math.asin(x/z) * 180/math.pi
        # yd = math.asin(y/z) * 180/math.pi
        # rospy.loginfo(xd)
        # rospy.loginfo(yd)
        # rospy.loginfo("Rounded xd: %s", round(xd, 1))
        # rospy.loginfo("Rounded yd: %s", round(yd, 1))
        rospy.loginfo("-----------------------------------------------------------------------------")
        rospy.loginfo(self.regions_angles['starboard_abeam_aft']['angles'][0])
        rospy.loginfo(self.regions_angles['starboard_abeam_aft']['parallel'])

    def parallelize(self) :
            # angles: [45.36971664428711, 44.63028335571289]
                # angles: [40.730220794677734, 49.269779205322266]

        # rospy.loginfo( self.regions_angles['starboard_abeam']['angles'][0] )
        # rospy.loginfo( self.regions_angles['starboard_abeam']['angles'][1] )
        # rospy.loginfo("[%s] <> [%s]: %s", self.regions_angles['starboard_abeam']['angles'][0], self.regions_angles['starboard_abeam']['angles'][1], np.isclose(self.regions_angles['starboard_abeam']['angles'][0], self.regions_angles['starboard_abeam']['angles'][1]))
        # if np.isclose(self.regions_angles['starboard_abeam']['angles'][0], self.regions_angles['starboard_abeam']['angles'][1], 0.75) :
        #     rospy.logerr("TARGET HIT!")
        #     # exit()
        # rospy.loginfo( np.isclose([self.regions_angles['starboard_abeam']['angles'][0], self.regions_angles['starboard_abeam']['angles'][1]], 10.0, False) )
        v1 = self.regions_angles['starboard_abeam']['angles'][0]
        v2 = self.regions_angles['starboard_abeam']['angles'][1]
        rospy.loginfo( "Status: %s, Values: [%s <> %s]", self.isclose(v1, v2), v1, v2 )
        if self.isclose(v1, v2, 0.1) and v1 != 0 and v2 != 0 :
            rospy.logwarn( "Status: %s, Values: [%s <> %s]", self.isclose(v1, v2), v1, v2 )

    def isclose(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def getTargetAngle(self, point) :
        return -3.14159274101 + (int(point) * 0.00579999992624)

    def getRotationSpeed(self, angle) :
        return self.kP * (angle - self.yaw)

    def rotationTarget(self, angle) :
        # target = self.getTargetAngle( angle )
        target = int(angle) * math.pi / 180 # to radians
        rospy.loginfo("Moving to %s degrees", math.degrees(target))
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        move = Twist()
        # if round(target, 2) == round(self.yaw, 2) :
        if self.isclose(target, self.yaw, 0.1) :
            move.linear.x = 0
            move.angular.z = 0
            pub.publish( move )
            rospy.logerr('Target Aqcuired!')
            exit()
        else :
        #     move.angular.z = self.getRotationSpeed(target)
            move.angular.z = self.getRotationSpeed(target)

        pub.publish( move )

    # def printBow(self) :
    #     # rospy.loginfo(self.regions['bow'])
    #     exit()

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
        print_args = sys.argv[1]
        scans_region = 'all'
        printData = True
    if "print=" in str(sys.argv) :
        print_args = sys.argv[1]
        scans_region = print_args.split('=')[1]
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
        laserReadingsDetected = True

    pid_turn = False
    if "pidTurn" in str(sys.argv) :
        pid_turn = True

    pidLine = False
    if "pidLine" in str(sys.argv) :
        pidLine = True

    pidRight = False
    if "pidRight" in str(sys.argv) :
        pidRight = True

    parallel = False
    if "parallel" in str(sys.argv) :
        parallel = True

    findrightwall = False
    if "findrightwall" in str(sys.argv) :
        findrightwall = True

    parallel_park = False
    if "parallel_park" in str(sys.argv) :
        parallel_park = True

    rotateToTarget = False
    if "rotateToTarget=" in str(sys.argv) :
        rotateToTarget_args = sys.argv[1]
        target_in_degrees = rotateToTarget_args.split('=')[1]
        rotateToTarget = True

    # printBow = False
    # if "printBow" in str(sys.argv) :
    #     printBow = True

    # rospy.Subscriber('/scan_front', LaserScan, rangeTester.callbackFront)
    # rospy.Subscriber('/scan_left', LaserScan, rangeTester.callbackLeft)
    # rospy.Subscriber('/scan_right', LaserScan, rangeTester.callbackRight)
    # rospy.Subscriber('/scan_center', LaserScan, rangeTester.callbackScans)
    # rospy.Subscriber('/scans_freespace', SpaceArray, rangeTester.callbackScansFreespace)
    rospy.Subscriber('/sweepbot/regions_angles', AnglesArray, rangeTester.callbckRegionsAngle)
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
                rangeTester.printData(scans_region)
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
            if pid_turn :
                rangeTester.pidTurn()
            if pidLine :
                rangeTester.pidLine()
            if getAngle :
                rangeTester.getAngle()
            if parallel :
                rangeTester.parallelize()
            if findrightwall :
                rangeTester.findrightwall()
            if parallel_park :
                rangeTester.parallel_park()
            if pidRight :
                rangeTester.pidRight()
            if rotateToTarget :
                rangeTester.rotationTarget(target_in_degrees)
            # if printBow :
            #     rangeTester.printBow()

        else :
            # rospy.loginfo("rangeTester.subR: %s, rangeTester.subFR: %s", rangeTester.subR, rangeTester.subF)
            rospy.loginfo("rangeTester.scans: %s", rangeTester.scans)

        rate.sleep()