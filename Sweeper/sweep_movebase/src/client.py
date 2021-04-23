#! /usr/bin/env python
from __future__ import print_function

import rospy
from sweepbot_tools.msg import TakePositionAction, TakePositionGoal
import actionlib
import sys

positions = {}

positions['east'] = {}
positions['east']['s0'] = [-40.6770896912, -44.9341926575, 0.00436401367188, 'junction 1']
positions['east']['s1'] = [-13.4710531235, -45.0709075928, 0.00177001953125, 'junction 2']
positions['east']['s2'] = [46.5702476501, -44.8143615723, 0.002197265625, 'junction 3']
positions['east']['s3'] = [123.651618958, -44.7183456421, 0.00332641601562, 'junction 4']

positions['mid'] = {}
positions['mid']['s0'] = [-43.205368042, 0.0783132314682, 0.00392913818359, 'junction 1']
positions['mid']['s1'] = [-13.8235702515, -0.0211445093155, 0.00350952148438, 'junction 2']
positions['mid']['s2'] = [46.207069397, 0.323795318604, 0.00392913818359, 'junction 3']
# positions['mid']['s3'] = [111.220985413, 0.0903835296631, 0.00337982177734, 'junction 1']
positions['mid']['s3'] = [121.419120789, 0.569704055786, 0.00166320800781, 'junction 4']

positions['west'] = {}
positions['west']['s0'] = [-43.063747406, 44.6970367432, 0.00392150878906, 'junction 1']
positions['west']['s1'] = [-13.2983016968, 44.9523048401, 0.00389862060547, 'junction 2']
positions['west']['s2'] = [46.7944450378, 44.9568824768, 0.00437164306641, 'junction 3']
positions['west']['s3'] = [121.479759216, 45.3283462524, 0.00157928466797, 'junction 4']
# positions['west']['s'] = []


def feedback_cb(msg):
    print( 'Received: ', msg)

def setPosition(street, junction):
    # Creates the SimpleActionClient, passing the type of the action
    rospy.loginfo( positions[street][junction] )
    client = actionlib.SimpleActionClient('sweepbot_movebase_server', TakePositionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = TakePositionGoal()
    goal.x = positions[street][junction][0]
    goal.y = positions[street][junction][1]
    goal.z = positions[street][junction][2]
    goal.position = street + ' street, ' + str(positions[street][junction][3])

    rospy.loginfo('Requesting navigation to ' + str(street) + ' street, ' + str(positions[street][junction][3]))
    # rospy.loginfo('Requesting navigation to ' + str(positions[street]) )

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('sweepbot_movebase_client')
        
        argv = rospy.myargv(sys.argv)
        rospy.loginfo(argv)
        
        if len(argv) != 3:
            rospy.logerr('Must specify 2 arguments [street] & [junction-id]. ' + str(len(argv)-1) + ' given')
        else:
            result = setPosition(argv[1], argv[2])
        
        # if len(sys.argv) < 1:
        #     print("usage: my_node.py {position}")
        # else:
        #     result = setPosition(sys.argv[1])
        #     # my_node(sys.argv[1], sys.argv[2])

        # result = setPosition()
        # rospy.logdebug(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)