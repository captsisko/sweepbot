#! /usr/bin/env python

import rospy
import actionlib

from sweepbot_tools.msg import TakePositionAction
# from sweepbot_tools.msg import TakePositionActionFeedback
# from sweepbot_tools.msg import TakePositionActionGoal
from sweepbot_tools.msg import TakePositionResult
from sweepbot_tools.msg import TakePositionFeedback
import sweepbot_tools.msg
# from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion
# from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SweepAction():
    _feedback = TakePositionFeedback()
    _result = TakePositionResult()

    def __init__(self, name):
        self._as = actionlib.SimpleActionServer(name, TakePositionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        if self._as.is_preempt_requested() :
            self._as.set_preempted()
            success = False
            return

        self.move(goal)

        feedback_obj = TakePositionFeedback()
        feedback_obj.report = 'server responding'
        self._as.publish_feedback(feedback_obj)

        result_obj = TakePositionResult()
        result_obj.completed = True
        self._as.set_succeeded(result_obj)

        r.sleep()

    def move(self, goal):
        rospy.loginfo(goal)
        client_mb = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client_mb.wait_for_server()

        # pose = Pose(Point(107.295516968, -0.0896091461182, 0.00246429443359), Quaternion(0.000, 0.000, 0.0, 1.0))
        pose = Pose(Point(goal.x, goal.y, goal.z), Quaternion(0.000, 0.000, 0.0, 1.0))
        
        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = pose
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        
        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(goal))
        feedback_obj = TakePositionFeedback()
        feedback_obj.report = 'Moving to: ' + str(goal.position)
        self._as.publish_feedback(feedback_obj)
        
        # Start the robot toward the next location
        # self.move_base.send_goal(self.goal)
        client_mb.send_goal(self.goal)
        client_mb.wait_for_result()
        
if __name__ == '__main__':
    rospy.init_node('sweepbot_movebase_server')
    server = SweepAction(rospy.get_name())
    rospy.spin()