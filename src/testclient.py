#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import roslib
import actionlib

import tbug.msg

def goal0_client():
	client=actionlib.SimpleActionClient('robot0_check_goals',tbug.msg.goalStatusAction)
	client.wait_for_server()
	goal = tbug.msg.goalStatusGoal(x=-5,y=11)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

if __name__=='__main__':
	try:
		rospy.init_node('test_client')
		result = goal0_client()
		print result.thereOrNot
	except rospy.ROSInterruptException:
		print "program interrupted before completion"