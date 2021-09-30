#!/usr/bin/env python

from pickle import FALSE, TRUE
import rospy
import math
from rospy.timer import sleep
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys
import numpy
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
    maneuver_complete = False
    previous_distance_to_wall = 0
    
    def __init__(self) :
        rospy.loginfo('Initializing ...')
        self.regions = {}
        self.current_state = -1
        self.sweeper_state = -1
        self.scans = 0
        self.imu = {}
        self.sweeper_states = CONFIG.STATES
        self.sweeper_move = Twist()
        self.debugMode = False
        self.median = -1000
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

    def getTargetAngle(self, point) :
        return -3.14159274101 + (point * 0.00579999992624)

    def getRotationSpeed(self, angle) :
        return self.kP * (angle - self.yaw)

    def clear_air(self) :
        if self.regions['port_bow'][0] == CONFIG.RANGE :
            return True
        else :
            return False
        
    def locate_right_wall(self) :
        move = Twist()

        rospy.logwarn_once("Initiataing right wall seeking")

        set_point = 1.0
        limit = 1.5

        distance_x = self.regions_angles[ CONFIG.starboard_active_section ]['mid_distance']
        pid = PID(0.12, 0.0, 0.0, set_point)
        pid_distance_x = pid( distance_x )

        distance_z = self.regions_angles[ CONFIG.starboard_active_section ]['mid_distance']
        pid = PID(0.1, 0.0, 0.0, set_point)
        pid_distance_z = pid( distance_z )

        move.linear.x = abs(pid_distance_x)
        
        if distance_z > limit :
            move.angular.z = -abs(pid_distance_z)

        if self.regions_angles['bow']['mid_distance'] < limit :
            move.linear.x = 0
            move.angular.z = 0
            self.change_state( self.sweeper_states['TURNLEFT'] )

        rospy.logwarn_throttle(10, ". . . still seeking right wall")
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

        rospy.logerr_once('Initiating LEFT-TURN')
        move = Twist()

        # if self.regions_angles['starboard_abeam']['angles'][0] < self.regions_angles['starboard_abeam']['angles'][1] :
        #     direction = -1
        # else :
        #     direction = 1

        if not self.regions_angles[ CONFIG.starboard_active_section ]['parallel'] :
            move.angular.z = 1
            # move.angular.z = direction
        else :
            rospy.logwarn('Robot is now parallel to wall!')
            self.change_state( self.sweeper_states['FOLLOWKERB'] )
            move.angular.z = 0

        rospy.logerr_throttle(5, '... turning')
        return move

    def follow_kerb(self) :

        move = Twist()

        distance_ahead = self.regions_angles['starboard_bow']['mid_distance']
        pid_ahead = PID(0.2, 0.3, 0.0, 6.0)
        pid_distance_ahead = pid_ahead(distance_ahead)
        move.linear.x = pid_distance_ahead

        distance_to_wall = self.regions_angles[ CONFIG.starboard_active_section ]['mid_distance']
        set_point = 1.0
        limit = 1.0
        pid_wall = PID(0.3, 0.3, 0.0, set_point)
        pid_distance_to_wall_z = pid_wall(distance_to_wall)

        if distance_to_wall != 8.0 :
            
            if self.previous_distance_to_wall == 0 :
                self.previous_distance_to_wall = distance_to_wall

            if not CONFIG.isclose(distance_to_wall, self.previous_distance_to_wall, 0.5) :
                rospy.loginfo("************* DISTANCE JUMP : %s to %s ---- resetting to %s", distance_to_wall, self.previous_distance_to_wall, self.previous_distance_to_wall)
                # distance_to_wall = self.previous_distance_to_wall
                move.linear.x = 0

                self.change_state( tasker.sweeper_states['TURNLEFT'] )
            else :

                if distance_to_wall > limit :
                    move.angular.z = -abs(pid_distance_to_wall_z)
                    rospy.logwarn("[Distance]: %s >>>>> [Prev]: %s [Closeness: %s]", distance_to_wall, self.previous_distance_to_wall, CONFIG.isclose(distance_to_wall, self.previous_distance_to_wall, 0.5))
                else :
                    move.angular.z = abs(pid_distance_to_wall_z)
                    rospy.logerr("[Distance]: %s <<<<< [Prev]: %s [Closeness: %s]", distance_to_wall, self.previous_distance_to_wall, CONFIG.isclose(distance_to_wall, self.previous_distance_to_wall, 0.5))

                self.previous_distance_to_wall = distance_to_wall

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
                # rospy.loginfo("----------------------------------------------------")
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
    rate = rospy.Rate(5) # Fixed update frequency of 10hz

    # arguments from cmd
    if "debug" in str(sys.argv) :
        tasker.debugMode = True

    rospy.Subscriber('/sweepbot/regions_angles', AnglesArray, tasker.callbckRegionsAngle)
    rospy.Subscriber('/scans_freespace', SpaceArray, tasker.callbackScansFreespace)
    rospy.Subscriber('/scans', LaserScan, tasker.callbackScans)
    rospy.Subscriber('/odom', Odometry, tasker.callbackOdometry)
    rospy.Subscriber('/imu', Imu, tasker.callbackIMU)

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
            else :
                tasker.sweeper_move = tasker.locate_right_wall()
            tasker.execute()

        rate.sleep()
        
    rospy.on_shutdown(tasker.shutdown)