#!/usr/bin/env python

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt32

# max range is the artificial value used as the maximum range of the laser scanner
maxRange = 3.0

# determines if the robot can see the goal directly
goalVisible = 0

# threshold that determines if a point in the laser scan is a discontinuity
discThresh = 3.0

scan = LaserScan()

def scanUpdate(lScan):
	global scan
	scan = lScan

def tangentbug():
	global goalVisible
	global scan
	goalfromlaser = rospy.Publisher('robot_2/robot2goal',Twist,queue_size=10)
	robotController = rospy.Publisher('robot_2/cmd_vel',Twist,queue_size=10)

	listener = tf.TransformListener()
	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		try:
			# trans is a list of 3 elements x,y,z of the transform. rot is a list of
			# 4 elements of the quaternion for the transform
			(trans,rot) = listener.lookupTransform('robot2/trueOdom','robot2/goal',rospy.Time(0))
			twi = Twist()
			twi.linear.x = trans[0]
			twi.linear.y = trans[1]
			twi.linear.z = trans[2]
			#ignore angular orientations since we only care about getting to the goal position
			twi.angular = Vector3(0.0,0.0,0.0)
			
			control = Twist()
			control.linear = Vector3(0.0,0.0,0.0)
			control.angular = Vector3(0.0,0.0,0.0)
			angle2goal = math.atan2(trans[1],trans[0])

			sensorIndex = int((angle2goal-scan.angle_min)/scan.angle_increment)

			# check if we can see the goal directly. Move towards it if we can
			if (scan.ranges[sensorIndex] >= maxRange):
				goalVisible = 1
				if (angle2goal > 0):
					control.linear.x = 0.3
					control.angular.z = 0.2
				elif (angle2goal < 0):
					control.linear.x = 0.3
					control.angular.z = -0.2
				else:
					control.linear.x = 0.3
					control.angular.z = 0.0
			else:
				goalVisible = 0

			# if we can't see the goal directly, check for the best direction of travel
			if goalVisible == 0:
				bestAngle = 0.0
				besti = 0
				bestDist = 10000.0
				realAngle = 0.0

				for i in range(len(scan.ranges)):
					# check for discontinuties within a specified threshold
					if (i>0) and (abs(scan.ranges[i]-scan.ranges[i-1]) > discThresh):
						# output the index for the discontinuity and the angle value and the distance to that discontinuity
						discDist = scan.ranges[i]
						if discDist==float('Inf'):
							discDist = scan.range_max
						dAng = scan.angle_min + i * scan.angle_increment
						xDist = discDist * math.sin(dAng)
						yDist = discDist * math.cos(dAng)
						heurDist = math.sqrt((twi.linear.x-xDist) ** 2 + (twi.linear.y-yDist) ** 2)
						if ((heurDist + discDist) < bestDist):
							bestDist = heurDist + discDist
							bestAngle = dAng
							besti = i

				# drive towards the best heuristic or turn towards it if we're not facing it already
				if ((bestAngle) > 0):
					control.linear.x = 0.2
					control.angular.z = 0.3
				elif ((bestAngle) < 0):
					control.linear.x = 0.2
					control.angular.z = -0.3
				else:
					control.linear.x = 0.2
					control.angular.z = 0.0

				# prioritize avoiding obstacles
				if (besti > 90) and (besti < (len(scan.ranges)-90)):
					if scan.ranges[besti+20] < 2.0:
						control.linear.x = 0.2
						control.angular.z = 0.5
					elif scan.ranges[besti-20] < 2.0:
						control.linear.x = 0.2
						control.angular.z = -0.5
				elif (besti > 90) and (besti < (len(scan.ranges)-90)):
					if scan.ranges[besti+30] < 2.0:
						control.linear.x = 0.2
						control.angular.z = 0.5
					elif scan.ranges[besti-30] < 2.0:
						control.linear.x = 0.2
						control.angular.z = -0.5

			# if obstacles are too close to the robot, prioritize avoiding them
			j = int(len(scan.ranges)/2) - 70
			m = int(len(scan.ranges)/2) - 10
			k = int(len(scan.ranges)/2) + 70
			n = int(len(scan.ranges)/2) + 10

			for i in range(j,m):
				if (scan.ranges[i] < 2.5):
					control.linear.x = 0.0
					control.angular.z = 0.5
			for i in range(n,k):
				if (scan.ranges[i] < 2.5):
					control.linear.x = 0.0
					control.angular.z = -0.5

			# stop moving if we're close enough to the goal
			if math.sqrt((twi.linear.x) ** 2 + (twi.linear.y) ** 2) < 0.5:
				control.linear.x = 0.0
				control.linear.y = 0.0
				
			robotController.publish(control)
			goalfromlaser.publish(twi)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	rate.sleep()

if __name__ == '__main__':
    try:
    	rospy.sleep(2.0)
    	rospy.init_node('tf_robot2_tbug')
    	rospy.Subscriber("robot_2/base_scan",LaserScan,scanUpdate)
    	tangentbug()
    	rospy.spin()
    except rospy.ROSInterruptException: pass