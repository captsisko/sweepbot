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

class Tasker :

    def __init__(self) :
        rospy.loginfo('Initializing')
        self.distance_to_wall = 0.5
        self.subL_value = -1
        self.subR_value = -1
        self.subFL_value = -1
        self.subFR_value = -1
        self.current_state = -1
        self.sweeper_state = -1
        self.sweeper_states = {
            # 'PARK' : 0,
            'LOCATE-RIGHT-WALL' : 1,
            'TURNLEFT' : 2,
            'FOLLOWKERB' : 3
        }
        self.sweeper_move = Twist

    def callbackLeft(self, msg) :
        self.subL_value = min(msg.ranges[0:22])

    def callbackRight(self, msg) :
        self.subR_value = min(msg.ranges[0:22])
        # rospy.loginfo(self.subR_value)
        # rospy.loginfo(msg.ranges[0:22])

    def callbackFront(self, msg) :
        self.subFR_value = min(msg.ranges[0:24])
        self.subFL_value = min(msg.ranges[24:49])

    def callbackOdometry(self, odom) :
        rospy.loginfo(odom.pose.pose)

    # def park(self) :
    #     self.sweeper_state = self.sweeper_states['LOCATE-RIGHT-WALL']
    #     rospy.loginfo('(park) Transitioning to %s: ', self.sweeper_state)
    #     return Twist()

    def locate_right_wall(self) :
        rospy.loginfo('---')
        move = Twist()
        move.linear.x = 0.5

        test_value = min(self.subR_value, self.subFR_value)
        rospy.loginfo("[Test-Value] Distance to right wall: %s ", test_value)

        if test_value > self.distance_to_wall :
            move.angular.z = -0.2
            # self.sweeper_state = self.sweeper_states['LOCATE-RIGHT-WALL']
        else :
            self.change_state( self.sweeper_states['TURNLEFT'] )
        rospy.loginfo("subFR_value: %s", self.subFR_value)
        rospy.loginfo("subR_value: %s", self.subR_value)
        return move

    def turn_left(self) :
        test_value = min(self.subR_value, self.subFR_value)
        rospy.loginfo("[Test-Value] Distance to right wall: %s ", test_value)

        move = Twist()

        combined_array = self.subFR + self.subR
        count = len(combined_array)
        total = sum( filter( self.reject_inf, combined_array ) )
        average = total/count
        rospy.loginfo('COUNT: %s', count)
        rospy.loginfo('TOTAL: %s', total)
        rospy.logwarn('AVERAGE: %s', average)

        if average > self.distance_to_wall :
            move.angular.z = 0.1
        elif average == self.distance_to_wall :
            move.angular.z = 0.0

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

    def execute(self) :
        kerb_tracker = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        kerb_tracker.publish( self.sweeper_move )

    def get_state_label(self, state_value) :
        for key, value in self.sweeper_states.items() :
            if value == state_value :
                return key

    def lasers_engaged(self) : # lasers minimum values populated and no longer the initial -1
        if self.subL_value != -1 and self.subR_value != -1 and self.subFL_value != -1 and self.subFR_value != -1 :
            return True

    def shutdown(self) :
        # self.park()
        rospy.logerr('FULL STOP!')
        rospy.logwarn('FULL STOP!')
        rospy.logfatal('FULL STOP!')

if __name__ == "__main__":
    rospy.init_node('wall_detector')
    
    tasker = Tasker()
    rate = rospy.Rate(10) # Fixed update frequency of 10hz

    rospy.Subscriber('/scan_front', LaserScan, tasker.callbackFront)
    rospy.Subscriber('/scan_left', LaserScan, tasker.callbackLeft)
    rospy.Subscriber('/scan_right', LaserScan, tasker.callbackRight)
    # rospy.Subscriber('/odom', Odometry, tasker.callbackOdometry)

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

        else :
            rospy.logwarn("Lasers still warming up")

        tasker.execute()
        rate.sleep()
        
    rospy.on_shutdown(tasker.shutdown)