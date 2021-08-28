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
import ast
import json

class Tasker :

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
        self.scans_fore = []
        # self.lasers_frontleft_merge = tuple()
        # self.lasers_frontright_merge = tuple()
        self.sweeper_states = {
            # 'PARK' : 0,
            'LOCATE-RIGHT-WALL' : 1,
            'FOLLOWKERB' : 3,
            'TURNLEFT' : 2,
            'DRIVE' : 4,
        }
        self.sweeper_move = Twist()
        self.debugMode = False
        self.median = -1000
        self.openspace_index = 0
        self.freespace = {}

        self.target = 0 # degrees to be applied to yaw
        self.kP = 0.5

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
        # rospy.loginfo(msg)
        # self.freespaces = msg
        # test = ast.literal_eval(msg.spaces[0].space)
        # rospy.loginfo( test['port_bow'] )
        # exit()
        for space in msg.spaces :
            # self.freespace[space] = ast.literal_eval(space.space)
            # self.freespace[space] = 'testing'
            rospy.loginfo( space[0] )
        
        # rospy.loginfo( self.freespace )
        # rospy.loginfo( json.dumps({'a':2, 'b':{'x':3, 'y':{'t1': 4, 't2':5}}}, sort_keys=True, indent=4) )
        # rospy.loginfo( json.dumps(self.freespace, sort_keys=True, indent=4) )


    def callbackOdometry(self, odom) :
        # rospy.loginfo(odom.pose.pose)
        orientation = odom.pose.pose.orientation
        self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        # rospy.logerr(yaw) # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        yaw_degrees = (yaw) * math.pi/180

    # def direction_check(self) :
    #     # self.sweeper_state = self.sweeper_states['LOCATE-RIGHT-WALL']
    #     self.change_state( self.sweeper_states['LOCATE-RIGHT-WALL'] )
    
    # Get rid of values outside the range of the lasers.
    # In this case, the values of 26.0 exceeds the 25.0 
    # laser limits and compromise the algorithm
    def clear_inf_values(self, set) :
        return tuple( item for item in set if item != 26.0 )

    def clean_set(self, set) :
        set_list = list(set)
        occurances = set_list.count(26.0)
        print('Occurances: %d' %(occurances))
        while set_list.count(26.0) :
            set_list.remove(26.0)
        return tuple(set_list)

    def getMedian(self, range) :
        # median = numpy.median( range[0]:range[1] )
        indexes = numpy.arange(range[0], range[1]+1)
        # rospy.loginfo(indexes)
        indexes_length = len(indexes)
        indexes_midpoint = indexes_length / 2
        # while self.scans[indexes_midpoint] == 26.0 :
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
        rospy.loginfo("Average: %s", range_average)
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
        
    def locate_right_wall(self) :
        move = Twist()

        if self.median == -1000 :
            self.median = self.getMedian([136, 407]) #'starboard_abeam_aft', 'starboard_abeam_bow'
        else :
            rospy.loginfo( "Median: %s", self.median )
            target = self.getTargetAngle(self.median)
            rospy.loginfo("Moving to %s degrees", math.degrees(target))

            if format(target, '.2f') == format(self.yaw, '.2f') :
                move.angular.z = 0
                rospy.logerr('Target Aqcuired!')
                self.target_acquired = True
                # exit()
                # self.sweeper_state = self.sweeper_states['TURNLEFT']
                self.change_state( self.sweeper_states['DRIVE'] )
            else :
                move.angular.z = self.getRotationSpeed(target)
                rospy.loginfo('Target: %s, Yaw: %s', target, self.yaw)

        return move

    def drive(self) :
        move = Twist()

        # 'starboard_bow' : self.scans[408:543],
        # 'port_bow' : self.scans[544:679],
         
        midRanges = self.getMedianRange(11, [408, 679]) # 
        midRangeAverage = self.getAverage(midRanges)

        rospy.loginfo('[DRIVE]Average: %s', midRangeAverage)

        if( midRangeAverage <= 1.5 ) :
            self.change_state( self.sweeper_states['TURNLEFT'] )
        else :
            move.linear.x = 0.5

        return move

    def turn_left(self) :
        move = Twist()

        rospy.logerr('READY TO TURN LEFT')

        # 'starboard_aft' : self.scans[0:135],
        # 'starboard_abeam_aft' : self.scans[136:271],
        # 'starboard_abeam_bow' : self.scans[272:407],
        # 'starboard_bow' : self.scans[408:543],
        # 'port_aft' : self.scans[949:1084],
        # 'port_abeam_aft' : self.scans[814:949],
        # 'port_abeam_bow' : self.scans[679:814],
        # 'port_bow' : self.scans[544:679],

        # rospy.loginfo(self.regions['port_abeam_aft'])
        # rospy.loginfo(self.regions['port_abeam_bow'])
        # rospy.loginfo(self.regions['port_abeam_aft'])
        # exit()

        port_regions = ['port_bow', 'port_abeam_bow', 'port_abeam_aft', 'port_aft']
        # spaces = self.freespaces.spaces.split(',')

        # rospy.loginfo( self.freespaces.spaces )
        # rospy.loginfo( self.freespaces.spaces['port_bow'].spaces )
        # exit()

        # for region in port_regions :
        #     for space in self.freespaces.spaces :
        #         rospy.loginfo( space )


        # for space in spaces :
        #     # **************************************************************************
        #     if int(space) :
        #         angle = self.getTargetAngle( space )
        #         speed = self.getRotationSpeed( angle ) * -1

        #         if format(angle, '.2f') == format(self.yaw, '.2f') :
        #             move.angular.z = 0
        #         else :
        #             move.angular.z = speed

        #         rospy.loginfo("Distance front of robot: %s", self.regions['port_bow'][0]) 
        #         if self.regions['port_bow'][0] != 26.0 :
        #             rospy.logerr("Obstacle ahead of robot")
        #         else :
        #             rospy.loginfo("Clear air ahead of robot")

        #         rospy.sleep(3) 
        #     # **************************************************************************

        return move

    def follow_kerb(self) :
        move = Twist()
        move.linear.x = 0.7
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

    rospy.Subscriber('/scans_freespace', SpaceArray, tasker.callbackScansFreespace)
    rospy.Subscriber('/scans', LaserScan, tasker.callbackScans)
    rospy.Subscriber('/odom', Odometry, tasker.callbackOdometry)

    # tasker.change_state( tasker.sweeper_states['LOCATE-RIGHT-WALL'] )
    tasker.change_state( tasker.sweeper_states['TURNLEFT'] )

    while not rospy.is_shutdown() :
        if tasker.lasers_engaged() :
            
            tasker.sweeper_move = Twist()
            if tasker.sweeper_state == tasker.sweeper_states['LOCATE-RIGHT-WALL'] : # 1 : 'locate right wall',
                tasker.sweeper_move = tasker.locate_right_wall()
            elif tasker.sweeper_state == tasker.sweeper_states['DRIVE'] : 
                tasker.sweeper_move = tasker.drive() # drive straight
            elif tasker.sweeper_state == tasker.sweeper_states['TURNLEFT'] : 
                tasker.sweeper_move = tasker.turn_left() # turn-left ==> follow-kerb
            elif tasker.sweeper_state == tasker.sweeper_states['FOLLOWKERB'] : 
                tasker.sweeper_move = tasker.follow_kerb() # turn-left ==> follow-kerb
            else :
                tasker.sweeper_move = tasker.locate_right_wall()
            tasker.execute()

        rate.sleep()
        
    rospy.on_shutdown(tasker.shutdown)