#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import roslib
import actionlib

import tbug.msg

robot0pos = Odometry();
robot1pos = Odometry();
robot2pos = Odometry();

def robot_0positionCheck(updatedState):
	global robot0pos
	robot0pos = updatedState

def robot_1positionCheck(updatedState):
	global robot1pos
	robot1pos = updatedState

def robot_2positionCheck(updatedState):
	global robot2pos
	robot2pos = updatedState


class GoalServer(object):
	global robot0pos
	global robot1pos
	global robot2pos

	_feedback = tbug.msg.goalStatusFeedback()
	_result =tbug.msg.goalStatusResult()

	def __init__(self):
		self._as = actionlib.SimpleActionServer('check_goals',tbug.msg.goalStatusAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

	def execute_cb(self, goal):
		robot0goalReached = 0
		robot1goalReached = 0
		robot2goalReached = 0
		r = rospy.Rate(1)

		if (math.sqrt((goal.x-robot0pos.pose.pose.position.x)**2 + (goal.y-robot0pos.pose.pose.position.y)**2)<0.5):
			robot0goalReached = 1

		if (math.sqrt((goal.x-robot1pos.pose.pose.position.x)**2 + (goal.y-robot1pos.pose.pose.position.y)**2)<0.5):
			robot0goalReached = 1

		if (math.sqrt((goal.x-robot2pos.pose.pose.position.x)**2 + (goal.y-robot2pos.pose.pose.position.y)**2)<0.5):
			robot2goalReached = 1

		self._feedback.x=robot0pos.pose.pose.position.x
		self._feedback.y=robot0pos.pose.pose.position.y

		self._as.publish_feedback(self._feedback)

		r.sleep()

		if (robot0goalReached):
			self._result.robot0_thereOrNot=1
		else:
			self._result.robot0_thereOrNot=0

		if (robot1goalReached):
			self._result.robot1_thereOrNot=1
		else:
			self._result.robot1_thereOrNot=0

		if (robot2goalReached):
			self._result.robot2_thereOrNot=1
		else:
			self._result.robot2_thereOrNot=0

		self._as.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node('goal0_server')
	rospy.Subscriber("robot_0/base_pose_ground_truth",Odometry,robot_0positionCheck)
	rospy.Subscriber("robot_1/base_pose_ground_truth",Odometry,robot_1positionCheck)
	rospy.Subscriber("robot_2/base_pose_ground_truth",Odometry,robot_2positionCheck)
	GoalServer()
	rospy.spin()
