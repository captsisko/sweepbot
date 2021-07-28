#!/usr/bin/env python

# from pickle import TRUE
# from yaml import serialize
import rospy
from rospy.core import loginfo
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf import transformations
from sensor_msgs.msg import LaserScan
import sys

class Tasker :

    def __init__(self) :
        rospy.loginfo('Initializing')
        self.sweeper_wall_buffer = 0.5
        self.current_state = -1
        self.sweeper_state = -1
        self.laser_left_minimum_value = -1
        self.laser_right_minimum_value = -1
        self.laser_frontleft_minimum_value = -1
        self.laser_frontright_minimum_value = -1
        self.laser_front_data = tuple()
        self.laser_left_data = tuple()
        self.laser_right_data = tuple()
        self.lasers_frontleft_merge = tuple()
        self.lasers_frontright_merge = tuple()
        self.sweeper_states = {
            # 'PARK' : 0,
            'LOCATE-RIGHT-WALL' : 1,
            'TURNLEFT' : 2,
            'FOLLOWKERB' : 3
        }
        self.sweeper_move = Twist()

        self.debugMode = False

    # collecting half the laser data from left-to-right i:e 0-90 degress
    # to simulate ?????????????????????
    def callbackLeft(self, msg) :
        self.laser_left_minimum_value = min(msg.ranges[0:22])
        self.laser_left_data = msg.ranges[0:22]

    # collecting half the laser data from left-to-right i:e 0-90 degress
    # to simulate ?????????????????????
    def callbackRight(self, msg) :
        self.laser_right_minimum_value = min(msg.ranges[0:22])
        self.laser_right_data = msg.ranges[0:22]
        # rospy.loginfo(self.laser_right_minimum_value)
        # rospy.loginfo(msg.ranges[0:22])
        # rospy.loginfo(msg.ranges)
        self.merge_lasers()

    # splits the front laser data down the middle for merging with the left and right laser data
    def callbackFront(self, msg) :
        self.laser_frontright_minimum_value = min(msg.ranges[0:24]) # left 0-90 degrees
        self.laser_frontleft_minimum_value = min(msg.ranges[24:49]) # right 90-180 degrees
        self.laser_front_data = msg.ranges
        self.merge_lasers()

    def callbackOdometry(self, odom) :
        rospy.loginfo(odom.pose.pose)

    def merge_lasers(self) :
        self.lasers_frontleft_merge  = self.laser_front_data[24:49] + self.laser_left_data
        self.lasers_frontright_merge = self.laser_front_data[0:24]  + self.laser_right_data

    def locate_right_wall(self) :
        # rospy.loginfo('---')
        move = Twist()
        move.linear.x = 0.5

        test_value = min(self.laser_right_minimum_value, self.laser_frontright_minimum_value)
        # rospy.loginfo("[Test-Value] Distance to right wall: %s ", test_value)

        if test_value > self.sweeper_wall_buffer :
            move.angular.z = -0.2
            # self.sweeper_state = self.sweeper_states['LOCATE-RIGHT-WALL']
        else :
            self.change_state( self.sweeper_states['TURNLEFT'] )
        # rospy.loginfo("laser_frontright_minimum_value: %s", self.laser_frontright_minimum_value)
        # rospy.loginfo("laser_right_minimum_value: %s", self.laser_right_minimum_value)
        return move

    def turn_left(self) :
        move = Twist()

        # combined_array = filter(self.reject_inf, self.lasers_frontright_merge)
        combined_array = self.lasers_frontright_merge

        rospy.loginfo('Testing . . . .')
        rospy.loginfo(combined_array)
        

        # if median != self.sweeper_wall_buffer :
        #     move.angular.z = 0.1
        # elif median == self.sweeper_wall_buffer :
        #     move.angular.z = 0.0

        return move

    def reject_inf(self, entry) :
        if not math.isinf(entry) :
            return True

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
        if self.laser_left_minimum_value != -1 and self.laser_right_minimum_value != -1 and self.laser_frontleft_minimum_value != -1 and self.laser_frontright_minimum_value != -1 and len(self.lasers_frontright_merge) != 0 :
            return True

    def shutdown(self) :
        rospy.logerr('FULL STOP!')
        rospy.logwarn('FULL STOP!')
        rospy.logfatal('FULL STOP!')

if __name__ == "__main__":
    rospy.init_node('wall_detector')
    
    tasker = Tasker()
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    # arguments from cmd
    if "debug=true" in str(sys.argv) :
        tasker.debugMode = True

    rospy.Subscriber('/scan_front', LaserScan, tasker.callbackFront)
    rospy.Subscriber('/scan_left', LaserScan, tasker.callbackLeft)
    rospy.Subscriber('/scan_right', LaserScan, tasker.callbackRight)
    # rospy.Subscriber('/odom', Odometry, tasker.callbackOdometry)
    tasker.merge_lasers()

    if not tasker.lasers_engaged : # used this delay to be certain we are getting real laser values and not -1
        rate.sleep(1)

    tasker.change_state(tasker.sweeper_states['LOCATE-RIGHT-WALL']) # set initial state of the robot

    while not rospy.is_shutdown():
        if tasker.lasers_engaged() :
            tasker.sweeper_move = Twist()
            if tasker.sweeper_state == tasker.sweeper_states['LOCATE-RIGHT-WALL'] : # 1 : 'locate right wall',
                tasker.sweeper_move = tasker.locate_right_wall()
            elif tasker.sweeper_state == tasker.sweeper_states['TURNLEFT'] : 
                tasker.sweeper_move = tasker.turn_left() # turn-left ==> follow-kerb
            # tasker.sweeper_move = tasker.turn_left()
        else :
            if tasker.debugMode :
                rospy.logwarn("Lasers still warming up")
                # rospy.logerr("self.laser_left_minimum_value: %s", tasker.laser_left_minimum_value)
                # rospy.logerr("self.laser_right_minimum_value: %s", tasker.laser_right_minimum_value)
                # rospy.logerr("self.laser_frontleft_minimum_value: %s", tasker.laser_frontleft_minimum_value)
                # rospy.logerr("self.laser_frontright_minimum_value: %s", tasker.laser_frontright_minimum_value)
                # rospy.logerr("len(self.lasers_frontright_merge): %s", len(tasker.lasers_frontright_merge))

        tasker.execute()
        rate.sleep()
        
    rospy.on_shutdown(tasker.shutdown)