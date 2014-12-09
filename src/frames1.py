#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist



def robot1frame(odom):
	br = tf.TransformBroadcaster()
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	z = odom.pose.pose.position.z
	qx = odom.pose.pose.orientation.x
	qy = odom.pose.pose.orientation.y
	qz = odom.pose.pose.orientation.z
	qw = odom.pose.pose.orientation.w
	br.sendTransform((x,y,z),(qx,qy,qz,qw),rospy.Time.now(),"robot1/trueOdom","robot1/world")
	
# creates a fixed goal frame called "goal" in tf 
def goalframe(twi):
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	x = twi.linear.x
	y = twi.linear.y
	br.sendTransform((x,y,0.0),(0.0,0.0,0.0,1.0),rospy.Time.now(),"robot1/goal","robot1/world")

if __name__ == '__main__':
    try:
    	rospy.init_node('tf_boadcaster1',anonymous=True)
    	rospy.Subscriber("/robot_1/base_pose_ground_truth",Odometry,robot1frame)
    	rospy.Subscriber("/robot_1/goal",Twist,goalframe)
    	rospy.spin()
    except rospy.ROSInterruptException: pass