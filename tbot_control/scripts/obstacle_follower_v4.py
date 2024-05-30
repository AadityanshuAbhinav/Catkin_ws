#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#import matplotlib import pyplot as plt




d0 = 0.2
dead_zone = 0.01
alpha_x = 2

dead_zone_theta = 15
alpha_theta = 0.04
#t_x_theta = np.empty([3,1000])
t_k_mat = [0.0]
dfront_k_mat = [0.0]


def callback(msg):
	global counter
	global x_k
	global theta_k
	
	if counter >0:
		x_kminus1 = x_k
		theta_kminus1 = theta_k
		print('x(k-1) = ', x_kminus1, ', x(k) = ', x_k)


	t_k_sec = msg.header.stamp.secs
	t_k_nsec = msg.header.stamp.nsecs
	t_k = t_k_sec + t_k_nsec*10**(-9)

	x_array_k = msg.ranges
	x_array_k_masked = np.ma.masked_equal(x_array_k, 0.0, copy=False)
	x_k = x_array_k_masked.min()

	theta_k = x_array_k.index(x_k)


	if abs(x_k-d0) > (dead_zone):
		move.linear.x = alpha_x*(x_k-d0)	
	else:
		move.linear.x = 0.0
	

	if theta_k > dead_zone_theta:
		if theta_k <= 180:
			move.angular.z = 0*alpha_theta*(theta_k)
		else:
			move.angular.z = 0*alpha_theta*(theta_k-359)
	else:
		move.angular.z = 0


	pub.publish(move)
	counter = counter + 1


if __name__ == '__main__':
	counter = 0

	rospy.init_node('obstacle_follower_b1')
	sub = rospy.Subscriber('tb3_1/scan', LaserScan, callback)
	pub = rospy.Publisher('tb3_1/cmd_vel', Twist)
	move = Twist()

	rospy.spin()
