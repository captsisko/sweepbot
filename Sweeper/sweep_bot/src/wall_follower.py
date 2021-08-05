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
import numpy
# from math import isclose

class Tasker :

    def __init__(self) :
        # rospy.Subscriber('/scan_front', LaserScan, self.callbackFront)
        # rospy.Subscriber('/scan_left', LaserScan, self.callbackLeft)
        # rospy.Subscriber('/scan_right', LaserScan, self.callbackRight)

        rospy.loginfo('Initializing ...')
        self.sweeper_wall_buffer = 0.5
        self.current_state = -1
        self.sweeper_state = -1
        self.laser_left_data_minimum  = 0
        self.laser_right_data_minimum = 0
        self.laser_front_data_minimum = 0
        self.laser_front_data = 0
        self.laser_left_data  = 0
        self.laser_right_data = 0
        # self.lasers_frontleft_merge = tuple()
        # self.lasers_frontright_merge = tuple()
        self.sweeper_states = {
            # 'PARK' : 0,
            'LOCATE-RIGHT-WALL' : 1,
            'TURNLEFT' : 2,
            'FOLLOWKERB' : 3
        }
        self.sweeper_move = Twist()
        self.debugMode = False

    def callbackLeft(self, msg) :
        self.laser_left_data = filter(self.replace_inf, msg.ranges)

    def callbackRight(self, msg) :
        self.laser_right_data = filter(self.replace_inf, msg.ranges)

    def callbackFront(self, msg) :
        self.laser_front_data = msg.ranges

    # def callbackOdometry(self, odom) :
    #     rospy.loginfo(odom.pose.pose)

    def direction_check(self) :
        self.laser_front_data_minimum = min(self.laser_front_data) if len(self.laser_front_data) > 0 else 0
        self.laser_right_data_minimum = min(self.laser_right_data) if len(self.laser_right_data) > 0 else 0
        self.laser_left_data_minimum  = min(self.laser_left_data)  if len(self.laser_left_data)  > 0 else 0
        if self.debugMode :
            rospy.loginfo("Front Data: %s", self.laser_front_data_minimum )
            rospy.loginfo("Right Data: %s", self.laser_right_data_minimum )
            rospy.loginfo("Left  Data: %s", self.laser_left_data_minimum  )
            rospy.loginfo('-----------------------------')
    
    # def min(self, iTuple) :
    #     # rospy.loginfo(iTuple)
    #     iList = list(iTuple)
    #     smallest = False
    #     for index in range(len(iList)) :
    #         if iList[index] != 0 :
    #             smallest = min( smallest, iList[index] )
    #     # rospy.loginfo(iList)
    #     return smallest

    def replace_inf(self, item) :
        if math.isinf( item ) :
            return False
        else :
            return True

    def locate_right_wall(self) :
        move = Twist()
        move.linear.x = 0.5

        rospy.loginfo("Checking: %s > %s", self.laser_front_data_minimum, self.sweeper_wall_buffer)

        if self.laser_front_data_minimum > self.sweeper_wall_buffer :
            move.angular.z = -0.2
        else :
            self.change_state( self.sweeper_states['TURNLEFT'] )
        
        return move

    def turn_left(self) :
        move = Twist()

        rospy.loginfo(self.laser_front_data)

        rospy.loginfo("Left Turn: %s", min(self.laser_front_data[22:26]))

        if min(self.laser_front_data[22:26]) != math.isinf :
            move.angular.z = 0.1
        elif min(self.laser_front_data[22:26]) == math.isinf :
            move.angular.z = 0.0

        return move

    # def reject_inf(self, entry) :
    #     if not math.isinf(entry) :
    #         return True

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
        if type(self.laser_left_data) == tuple and type(self.laser_right_data) == tuple and type(self.laser_front_data) == tuple :
            if self.debugMode : 
                rospy.logwarn("laser_front_data: %s,[empty=%s]", min(self.laser_front_data) > 0 if len(self.laser_front_data) else 0, self.laser_front_data==tuple())
                rospy.logwarn("laser_right_data: %s,[empty=%s]", min(self.laser_right_data) > 0 if len(self.laser_right_data) else 0, self.laser_right_data==tuple())
                rospy.logwarn("laser_left_data:  %s,[empty=%s]", min(self.laser_left_data)  > 0 if len(self.laser_left_data)  else 0, self.laser_left_data==tuple())
                rospy.logwarn("-----------------------------------------------")
            return True
        elif type(self.laser_left_data) == int and type(self.laser_right_data) == int and type(self.laser_front_data) == int :
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

    rospy.Subscriber('/scan_front', LaserScan, tasker.callbackFront)
    rospy.Subscriber('/scan_left',  LaserScan, tasker.callbackLeft)
    rospy.Subscriber('/scan_right', LaserScan, tasker.callbackRight)

    while not rospy.is_shutdown() :
        if tasker.lasers_engaged() :

            tasker.direction_check()
            
            tasker.sweeper_move = Twist()
            if tasker.sweeper_state == tasker.sweeper_states['LOCATE-RIGHT-WALL'] : # 1 : 'locate right wall',
                tasker.sweeper_move = tasker.locate_right_wall()
            elif tasker.sweeper_state == tasker.sweeper_states['TURNLEFT'] : 
                tasker.sweeper_move = tasker.turn_left() # turn-left ==> follow-kerb
            elif tasker.sweeper_state == tasker.sweeper_states['FOLLOWKERB'] : 
                tasker.sweeper_move = tasker.follow_kerb() # turn-left ==> follow-kerb
            else :
                tasker.sweeper_move = tasker.locate_right_wall()
            tasker.execute()

            # rate.sleep()
        rate.sleep()
        
    rospy.on_shutdown(tasker.shutdown)