#!/usr/bin/env python

import rospy
import math
# from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan

def sample(data, SIDE):
    # print "--------------" + SIDE + "--------------"
    # print data.ranges
    cleanedData = filter(lambda x: not math.isinf(x), data.ranges)
    # print sum(cleanedData)
    print SIDE + ' Average: ' + str(sum(cleanedData)/50)
    # print len(cleanedData)
    # print sum(cleanedData)/50
    # print "----------------------------"

def setUp():
    rospy.init_node('lidar_test', anonymous=False)
    rospy.Subscriber('scan_left', LaserScan, sample, 'LEFT')
    rospy.Subscriber('scan_right', LaserScan, sample, 'RIGHT')
    rospy.spin()

def move():
    # locations['hall_foyer'] = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
    # test = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
    test = Pose(Point(107.295516968, -0.0896091461182, 0.00246429443359), Quaternion(0.000, 0.000, 0.0, 0.0))
    # Set up the next goal location
    self.goal = MoveBaseGoal()
    self.goal.target_pose.pose = test
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    
    # Let the user know where the robot is going next
    rospy.loginfo("Going to: " + str(test))
    
    # Start the robot toward the next location
    self.move_base.send_goal(self.goal)

if __name__ == "__main__":
    move()
# 
    # try:
    #     while not rospy.is_shutdown():
    #     # print "Testing ..."
    # except rospy.ROSInterruptException:
    #     pass