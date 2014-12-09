#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import roslib
import actionlib

import tbug.msg

currentGoal = Vector3(-5.0,11.0,0.0)

def goal0_client():
	global currentGoal

	client=actionlib.SimpleActionClient('check_goals',tbug.msg.goalStatusAction)
	client.wait_for_server()
	goal = tbug.msg.goalStatusGoal(x=currentGoal.x,y=currentGoal.y)
	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

# creates a fixed goal frame in tf 
def goalPub():
	global currentGoal
	goalCount = 0

	goalinworld = rospy.Publisher('robot_0/goal',Twist,queue_size=10)
	goal = Twist()
	goal.linear = Vector3(currentGoal.x,currentGoal.y,0.0)
	goal.angular = Vector3(0.0,0.0,0.0)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		result = goal0_client()
		if result.robot0_thereOrNot == 1:
			goalCount = goalCount + 1

		if goalCount>=1:
			currentGoal.x = 0.0
			currentGoal.y = 0.0
		else:
			currentGoal.x = -5.0
			currentGoal.y = 11.0

		goal.linear = Vector3(currentGoal.x,currentGoal.y,0.0)
		goal.angular = Vector3(0.0,0.0,0.0)

		rospy.loginfo(goalCount)
		rospy.loginfo(goal.linear)
		goalinworld.publish(goal)
		rate.sleep()

if __name__ == '__main__': 
    try:
    	rospy.init_node("robot0_goal")
    	goalPub()
    except rospy.ROSInterruptException: pass