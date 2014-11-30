#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
	
# creates a fixed goal frame in tf 
def goalPub():
	goalinworld = rospy.Publisher('robot_1/goal',Twist,queue_size=10)
	goal = Twist()
	goal.linear = Vector3(11.0,0.0,0.0)
	goal.angular = Vector3(0.0,0.0,0.0)
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		goalinworld.publish(goal)
		rate.sleep()

if __name__ == '__main__': 
    try:
    	rospy.init_node("robot1_goal")
    	goalPub()
    except rospy.ROSInterruptException: pass