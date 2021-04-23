#!/usr/bin/env python

import rospy
import math
# from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def sample(data, SIDE):
    # print "--------------" + SIDE + "--------------"
    # print data.ranges
    cleanedData = filter(lambda x: not math.isinf(x), data.ranges)
    # print sum(cleanedData)
    print SIDE + ' Average: ' + str(sum(cleanedData)/50)
    # print len(cleanedData)
    # print sum(cleanedData)/50
    # print "----------------------------"

def move():
    # locations['hall_foyer'] = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
    # test = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
    test = Pose(Point(107.295516968, -0.0896091461182, 0.00246429443359), Quaternion(0.000, 0.000, 0.0, 1.0))

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = 0.5
    # goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.pose = test

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def setUp():
    rospy.init_node('lidar_test', anonymous=False)
    # rospy.Subscriber('scan_left', LaserScan, sample, 'LEFT')
    # rospy.Subscriber('scan_right', LaserScan, sample, 'RIGHT')
    move()
    rospy.spin()

if __name__ == "__main__":
    # setUp()
    try:
        rospy.init_node('movebase_client_py')
        result = move()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
# 
    # try:
    #     while not rospy.is_shutdown():
    #     # print "Testing ..."
    # except rospy.ROSInterruptException:
    #     pass