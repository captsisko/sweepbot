#!/usr/bin/env python

# from pickle import TRUE
# from yaml import serialize
import rospy
# from rospy.core import loginfo
import math
from rospy.timer import sleep
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf import transformations
from sensor_msgs.msg import LaserScan
import sys
import numpy
# from math import isclose
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from sweep_bot.msg import Space
from sweep_bot.msg import SpaceArray
from sensor_msgs.msg import Imu
from simple_pid import PID
from sweep_bot.msg import AngleArray
from sweep_bot.msg import AnglesArray
import settings as CONFIG

class Tasker :
    regions_angles = {}
    RANGE = 6.0

    def __init__(self) :
        rospy.loginfo('Initializing ...')
        self.regions = {}
        self.sweeper_wall_buffer = 0.5
        self.current_state = -1
        self.sweeper_state = -1
        # self.laser_left_data_minimum  = 0
        # self.laser_right_data_minimum = 0
        # self.laser_front_data_minimum = 0
        self.scans_minimum_left = 0
        self.scans_minimum_right = 0
        self.scans_minimum_fore = 0
        self.scans_minimum_rear = 0
        # self.laser_front_data = 0
        # self.laser_left_data  = 0
        # self.laser_right_data = 0
        self.scans = 0
        self.imu = {}
        self.scans_fore = []
        # self.lasers_frontleft_merge = tuple()
        # self.lasers_frontright_merge = tuple()
        self.sweeper_states = CONFIG.STATES
        self.sweeper_move = Twist()
        self.debugMode = False
        self.median = -1000
        self.openspace_index = 0
        self.freespaces = {}

        self.target = 0 # degrees to be applied to yaw
        self.kP = 0.5
        self.rotation_successful = False

        self.gap = -1

    def callbackLeft(self, msg) :
        self.laser_left_data = filter(self.replace_inf, msg.ranges)

    def callbackRight(self, msg) :
        self.laser_right_data = filter(self.replace_inf, msg.ranges)

    def callbackFront(self, msg) :
        self.laser_front_data = msg.ranges

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
        }

    def callbackScansFreespace(self, msg) :
        for spaces in msg.spaces :
            # rospy.loginfo(spaces)
            self.freespaces[spaces.region] = spaces.gap

    def yaw_from_euler_from_quaternion(self, orientation) :
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        return (yaw) * math.pi/180

    def callbackOdometry(self, odom) :
        # orientation = odom.pose.pose.orientation
        # self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        # (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        # self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        # yaw_degrees = (yaw) * math.pi/180

        # to be refactored to use yaw_from_euler_from_quaternion() function
        orientation = odom.pose.pose.orientation
        self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        yaw_degrees = (yaw) * math.pi/180

    def callbckRegionsAngle(self, anglesArray) :
        for regionAngles in anglesArray.angles :
            self.regions_angles[regionAngles.region] = {
                'angles' : regionAngles.angles,
                'parallel' : regionAngles.parallel,
                'mid_distance' : regionAngles.mid_distance,
                'avg_distance' : regionAngles.avg_distance
            }

    def callbackIMU(self, imu) :
        # rospy.loginfo( imu )
        # self.imu['orientation'] = imu.orientation
        self.imu['orientation'] = imu

    # def direction_check(self) :
    #     # self.sweeper_state = self.sweeper_states['LOCATE-RIGHT-WALL']
    #     self.change_state( self.sweeper_states['LOCATE-RIGHT-WALL'] )
    
    # Get rid of values outside the range of the lasers.
    # In this case, the values of CONFIG.RANGE exceeds the 25.0 
    # laser limits and compromise the algorithm
    def clear_inf_values(self, set) :
        return tuple( item for item in set if item != CONFIG.RANGE )

    def clean_set(self, set) :
        set_list = list(set)
        occurances = set_list.count(CONFIG.RANGE)
        # print('Occurances: %d' %(occurances))
        while set_list.count(CONFIG.RANGE) :
            set_list.remove(CONFIG.RANGE)
        return tuple(set_list)

    def getMedian(self, range) :
        # median = numpy.median( range[0]:range[1] )
        indexes = numpy.arange(range[0], range[1]+1)
        # rospy.loginfo(indexes)
        indexes_length = len(indexes)
        indexes_midpoint = indexes_length / 2
        # while self.scans[indexes_midpoint] == CONFIG.RANGE :
        #     indexes_midpoint -= 1
        return indexes_midpoint
        # **********************************************************
        # median = numpy.median( self.clear_inf_values(self.scans[range[0]:range[1]]) )
        # rospy.loginfo( "MEDIAN INDEX: %s", median )
        # rospy.loginfo( "MEDIAN VALUE: %s", self.scans.index(median) )
        # return self.scans.index(median)

    def getMedianRange(self, range, limits=[33,66]) :
        arr = self.scans[ limits[0] : limits[1] ]
        arr_length = len(arr)
        
        median = arr_length / 2

        # rospy.loginfo("RANGE: %s", range)
        # rospy.loginfo("LIMITS: %s", limits)
        # rospy.loginfo('Median: %s => value: %s', median, arr[median])
        # rospy.loginfo("TYPE: %s", type( int(range) ))
        # rospy.loginfo("ARR: %s", arr )

        range = int(range)
        start = stop = (range-1) / 2

        # rospy.loginfo("start: %s, stop: %s", start, stop)

        data = self.clean_set( arr[median-start : median+stop+1] )

        # rospy.loginfo( data )

        return data

    # def getAverage(self, range) :
    #     range_list = self.clean_set( list( self.scans[range[0] : range[1]] ) )
    #     range_average = numpy.average(range_list)
    #     rospy.loginfo("Average: %s", range_average)
    #     return range_average
    
    def getAverage(self, list) :
        # range_list = self.clean_set( list( self.scans[range[0] : range[1]] ) )
        range_average = numpy.average(list)
        # rospy.loginfo("Average: %s", range_average)
        return range_average

    def getMin(self, tuple1, tuple2) :
        value1 = min( self.clear_inf_values(tuple1) )
        value2 = min( self.clear_inf_values(tuple2) )
        return min(value1, value2)

    def getMax(self, tuple1, tuple2) :
        value1 = max( self.clear_inf_values(tuple1) )
        value2 = max( self.clear_inf_values(tuple2) )
        return max(value1, value2)

    def getTargetAngle(self, point) :
        return -3.14159274101 + (point * 0.00579999992624)

    def getRotationSpeed(self, angle) :
        return self.kP * (angle - self.yaw)

    def clear_air(self) :
        if self.regions['port_bow'][0] == CONFIG.RANGE :
            return True
        else :
            return False
        
    # def locate_right_wall(self) :
    #     move = Twist()

    #     if self.median == -1000 :
    #         self.median = self.getMedian([136, 407]) #'starboard_abeam_aft', 'starboard_abeam_bow'
    #     else :
    #         rospy.loginfo( "Median: %s", self.median )
    #         target = self.getTargetAngle(self.median)
    #         rospy.loginfo("Moving to %s degrees", math.degrees(target))

    #         if format(target, '.2f') == format(self.yaw, '.2f') :
    #             move.angular.z = 0
    #             rospy.logerr('Target Aqcuired!')
    #             self.target_acquired = True
    #             # exit()
    #             # self.sweeper_state = self.sweeper_states['TURNLEFT']
    #             self.change_state( self.sweeper_states['DRIVE'] )
    #         else :
    #             move.angular.z = self.getRotationSpeed(target)
    #             rospy.loginfo('Target: %s, Yaw: %s', target, self.yaw)

    #     return move

    def locate_right_wall(self) :
        move = Twist()

        rospy.logwarn("Initiataing right wall seeking")
        # ----------------------------------------------------------------------------------------
        # pid = PID(0.15, 0.3, 0.0, 1.5)
        # longest_distance_x = self.regions['starboard_abeam_bow'][0]
        # pid_longest_distance_x = round( pid(longest_distance_x), 2 )
        # rospy.loginfo("Distance-X: %s, PID: %s", longest_distance_x, pid_longest_distance_x)

        # pid = PID(0.01, 0.1, 0.0, 1.5)
        # longest_distance_z = self.regions['starboard_abeam_aft'][-1]
        # pid_longest_distance_z = round( pid(longest_distance_z), 2 )
        # rospy.loginfo("Distance-Z: %s, PID: %s", longest_distance_z, pid_longest_distance_z)

        # if longest_distance_x > 1.5 and longest_distance_z > 1.5 :
        #     move.linear.x = pid_longest_distance_x * -1
        #     move.angular.z -= pid_longest_distance_z * -1
        #     rospy.loginfo(move)
        # else :
        #     rospy.logwarn("Terminating right wall seeking")
        #     move.linear.x = 0
        #     move.angular.z -= 0
            
        #     # self.change_state( self.sweeper_states['DRIVE'] )
        #     self.change_state( self.sweeper_states['TURNLEFT'] )
        # ----------------------------------------------------------------------------------------

        distance_to_wall = self.regions_angles['starboard']['avg_distance']
        pid = PID(1.0, 0.0, 0.0, 0.5)
        pid_distance_to_wall = pid(distance_to_wall)

        move.linear.x = 0.5

        if distance_to_wall < 1.0 :
            move.angular.z -= pid_distance_to_wall
            rospy.logerr("%s >>>>>", distance_to_wall)

        if self.regions_angles['bow']['mid_distance'] < 1.5 :
            self.change_state( self.sweeper_states['TURNLEFT'] )
            move.linear.x = 0
            move.angular.z = 0
        return move

    # def drive(self) :
    #     move = Twist()

    #     # 'starboard_bow' : self.scans[408:543],
    #     # 'port_bow' : self.scans[544:679],
         
    #     midRanges = self.getMedianRange(11, [408, 679]) # 
    #     midRangeAverage = self.getAverage(midRanges)

    #     rospy.loginfo('[DRIVE]Average: %s', midRangeAverage)

    #     if( midRangeAverage <= 1.5 ) :
    #         self.change_state( self.sweeper_states['TURNLEFT'] )
    #     else :
    #         move.linear.x = 0.5

    #     return move

    def turn_left(self) :

        # if self.gap == -1 :
        #     self.gap = self.freespaces['port']
        #     rospy.loginfo("GAP set to : %s", self.gap)
        #     rospy.sleep(5)

        move = Twist()

        rospy.logerr('READY TO TURN LEFT')

        rospy.loginfo( self.regions_angles['starboard_abeam'] )

        if not self.regions_angles['starboard_abeam']['parallel'] :
            move.angular.z = 1
        else :
            rospy.logwarn('Robot is now parallel to wall!')
            self.change_state( self.sweeper_states['FOLLOWKERB'] )
            move.angular.z = 0

        # self.gap = int(self.gap)
        # rospy.loginfo( "GAP: %s", self.gap )
        # target = self.getTargetAngle( self.gap )
        # rospy.loginfo("Moving to %s degrees", math.degrees(target))

        # if format(target, '.2f') == format(self.yaw, '.2f') :
        #     move.angular.z = 0
        #     rospy.logerr('Target Aqcuired!')
        #     # self.target_acquired = True
        #     if not self.clear_air() :
        #         # rospy.logwarn('Non-Clean Air Ahead!')
        #         self.change_state( self.sweeper_states['TURNLEFT'] )
        #         self.gap += 20
        #     else :
        #         # rospy.logwarn('Clean Air Ahead!')
        #         self.change_state( self.sweeper_states['FOLLOWKERB'] )
        # else :
        #     move.angular.z = self.getRotationSpeed(target)
            # rospy.loginfo('Target: %s, Yaw: %s', target, self.yaw)

        return move

    def follow_kerb(self) :
        move = Twist()

        distance_to_wall = self.regions_angles['starboard']['avg_distance']
        set_point = 1.5
        pid = PID(0.3, 0.1, 0.0, set_point)
        pid_distance_to_wall_z = round( pid(distance_to_wall), 2 )

        move = Twist()
        move.linear.x = 0.5

        if distance_to_wall < set_point :
            move.angular.z += pid_distance_to_wall_z
            rospy.logwarn("%s <<<<< %s", distance_to_wall, pid_distance_to_wall_z)
        else :
            move.angular.z -= pid_distance_to_wall_z * -1
            rospy.logerr("%s >>>>> %s", distance_to_wall, pid_distance_to_wall_z * -1)
        
        return move

    def change_state(self, previous_state) :
        if self.sweeper_state is not previous_state :
            rospy.loginfo(
                "%s[%s] changed to %s[%s]", 
                self.get_state_label(self.sweeper_state), 
                self.sweeper_state, 
                self.get_state_label(previous_state),
                previous_state
            )
            self.sweeper_state = previous_state

    # execute twist entity
    def execute(self) :
        kerb_tracker = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        kerb_tracker.publish( self.sweeper_move )

    # returns state keys to make debug output more readable
    def get_state_label(self, state_value) :
        for key, value in self.sweeper_states.items() :
            if value == state_value :
                return key

    # checking lasers minimum values are populated and no longer the initial -1
    def lasers_engaged(self) : 
        if type(self.scans) == tuple : #and type(self.laser_right_data) == tuple : # and type(self.laser_front_data) == tuple :
            if self.debugMode : 
                rospy.logwarn("scans:  %s,[empty=%s]", min(self.scans)  > 0 if len(self.scans)  else 0, self.scans==tuple())
                rospy.logwarn("-----------------------------------------------")
            return True
        elif type(self.scans) == int : #and type(self.laser_right_data) == int : #and type(self.laser_front_data) == int :
            if self.debugMode :
                rospy.logfatal("NO LASER DATA : Lasers warming up")
            return False

    def shutdown(self) :
        rospy.logerr('FULL STOP!')
        rospy.logwarn('FULL STOP!')
        rospy.logfatal('FULL STOP!')

if __name__ == "__main__":
    rospy.init_node('wall_detector')
    
    tasker = Tasker()
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    # arguments from cmd
    if "debug" in str(sys.argv) :
        tasker.debugMode = True

    rospy.Subscriber('/sweepbot/regions_angles', AnglesArray, tasker.callbckRegionsAngle)
    rospy.Subscriber('/scans_freespace', SpaceArray, tasker.callbackScansFreespace)
    rospy.Subscriber('/scans', LaserScan, tasker.callbackScans)
    rospy.Subscriber('/odom', Odometry, tasker.callbackOdometry)
    rospy.Subscriber('/imu', Imu, tasker.callbackIMU)

    # if "left" in str(sys.argv) :
    #     tasker.change_state( tasker.sweeper_states['TURNLEFT'] )
    # if "follow" in str(sys.argv) :
    #     tasker.change_state( tasker.sweeper_states['FOLLOWKERB'] )
    # else :
    #     tasker.change_state( tasker.sweeper_states['LOCATE-RIGHT-WALL'] )

    while not rospy.is_shutdown() :
        if tasker.lasers_engaged() :
            
            tasker.sweeper_move = Twist()
            if tasker.sweeper_state == tasker.sweeper_states['LOCATE-RIGHT-WALL'] : # 1 : 'locate right wall',
                tasker.sweeper_move = tasker.locate_right_wall()
            # elif tasker.sweeper_state == tasker.sweeper_states['DRIVE'] : 
            #     tasker.sweeper_move = tasker.drive() # drive straight
            elif tasker.sweeper_state == tasker.sweeper_states['TURNLEFT'] : 
                tasker.sweeper_move = tasker.turn_left() # turn-left ==> follow-kerb
            elif tasker.sweeper_state == tasker.sweeper_states['FOLLOWKERB'] : 
                tasker.sweeper_move = tasker.follow_kerb() # turn-left ==> follow-kerb
                # tasker.follow_kerb() # turn-left ==> follow-kerb
                # rospy.loginfo("----------------------------------------------------")
            else :
                tasker.sweeper_move = tasker.locate_right_wall()
            tasker.execute()

        rate.sleep()
        
    rospy.on_shutdown(tasker.shutdown)