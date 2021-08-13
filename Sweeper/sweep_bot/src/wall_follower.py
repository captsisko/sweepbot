#!/usr/bin/env python

# from pickle import TRUE
# from yaml import serialize
import rospy
# from rospy.core import loginfo
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
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

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
            'TURNLEFT' : 2,
            'FOLLOWKERB' : 3
        }
        self.sweeper_move = Twist()
        self.debugMode = False
        self.median = -1000

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
                'abeam'   : {

                },
            },
            'starboard_aft' : self.scans[0:135],
            'starboard_abeam_aft' : self.scans[136:271],
            'starboard_abeam_bow' : self.scans[272:407],
            'starboard_bow' : self.scans[408:543],
            'port_aft' : self.scans[949:1084],
            'port_abeam_aft' : self.scans[814:949],
            'port_abeam_bow' : self.scans[679:814],
            'port_bow' : self.scans[544:679],
        }

    def callbackOdometry(self, odom) :
        # rospy.loginfo(odom.pose.pose)
        orientation = odom.pose.pose.orientation
        self.orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
        # rospy.logerr(yaw) # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        self.yaw = yaw # this yaw value is in radians NOT DEGREES with respect to the world coordinates
        yaw_degrees = (yaw) * math.pi/180

    def direction_check(self) :
        # port_aft = min(self.regions['port_abeam_aft'])
        # port_bow = min(self.regions['port_abeam_bow'])
        # rospy.loginfo("PORT-Aft-min: %s ", port_aft)
        # rospy.loginfo("PORT-Bow-min: %s ", port_bow)
        # rospy.loginfo("PORT-Min: %s ", min(port_aft, port_bow))

        # starboard_aft = min(self.regions['starboard_abeam_aft'])
        # starboard_bow = min(self.regions['starboard_abeam_bow'])
        # rospy.loginfo("STarbord-Aft-min: %s ", starboard_aft)
        # rospy.loginfo("STarbord-Bow-min: %s ", starboard_bow)
        # rospy.loginfo("STarbord-Min: %s ", min(starboard_aft, starboard_bow))

        self.sweeper_state = self.sweeper_states['LOCATE-RIGHT-WALL']
        # rospy.loginfo('-----------------------------')
        
        # self.scans_minimum  = min(self.scans)  if len(self.scans)  > 0 else 0
        # if self.debugMode :
        #     rospy.loginfo("Left  Data: %s", self.scans_minimum  )
        #     rospy.loginfo('-----------------------------')
        # exit()
    
    # Get rid of values outside the range of the lasers.
    # In this case, the values of 26.0 exceeds the 25.0 
    # laser limits and compromise the algorithm
    def clear_inf_values(self, set) :
        return tuple( item for item in set if item != 26.0 )

    def getMedian(self, range) :
        # rospy.loginfo("SIZE: %s", len(self.scans))
        # return tuple( item for item in set if item != 26.0 )
        median = numpy.median( self.scans[range[0]:range[1]] )
        rospy.loginfo( "MEDIAN INDEX: %s", self.scans.index(median) )
        # if numpy.median(range) :
        #     return numpy.median(range)
        return self.scans.index(median)

    def getMin(self, tuple1, tuple2) :
        value1 = min( self.clear_inf_values(tuple1) )
        value2 = min( self.clear_inf_values(tuple2) )
        return min(value1, value2)

    def getMax(self, tuple1, tuple2) :
        value1 = max( self.clear_inf_values(tuple1) )
        value2 = max( self.clear_inf_values(tuple2) )
        return max(value1, value2)

    def locate_right_wall(self) :
        move = Twist()
        # move.linear.x = 0.5

        if self.median == -1000 :
            self.median = self.getMedian([136, 407]) #'starboard_abeam_aft', 'starboard_abeam_bow'
            # exit()
        else :
            rospy.loginfo( "Median: %s", self.median )
        
            # heading = -3.14159274101 + (self.median * 0.00579999992624)
            heading = 0
            # for i in range(self.median) :
            #     heading += -3.14159274101 + (i * 0.00579999992624)
                # heading += -3.14 + (i * 0.005)
            # heading = -3.14 + (200 * 0.005)
            # heading = -3.14 + (self.median * 0.005)
            heading = -3.14159274101 + (self.median * 0.00579999992624)
            # heading = -3.14159274101
            # heading = 0
            # heading = self.median * 0.00579999992624

            # ***************************************************************************************

            rospy.loginfo("HEADING: %s", heading)
            
            # # target = heading * math.pi/180 # converting degrees into radians
            # target = math.radians(heading)
            target = heading
            rospy.loginfo("TARGET: %s", target)
            if format(target, '.2f') == format(self.yaw, '.2f') :
                move.angular.z = 0
                rospy.logerr('Target Aqcuired!')
                self.target_acquired = True
                exit()
            else :
                move.angular.z = self.kP * (target - self.yaw)
                rospy.loginfo('Target: %s, Yaw: %s', target, self.yaw)

            # ***************************************************************************************

            # return move


            # if maxi > self.sweeper_wall_buffer :
            #     move.angular.z = -0.2
            # else :
            #     move.linear.x = 0
            #     move.angular.z = 0
            #     # self.change_state( self.sweeper_states['TURNLEFT'] )
        
        return move

    def turn_left(self) :
        move = Twist()

        # rospy.loginfo(self.laser_right_data)

        mini = min(min(self.regions['starboard_bow']), min(self.regions['port_bow']))
        rospy.loginfo("Left Turn: %s", mini)

        # if min(self.laser_front_data[22:26]) != math.isinf :
        #     move.angular.z = 0.1
        # elif min(self.laser_front_data[22:26]) == math.isinf :
        #     move.angular.z = 0.0

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

    rospy.Subscriber('/scans', LaserScan, tasker.callbackScans)
    rospy.Subscriber('/odom', Odometry, tasker.callbackOdometry)

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