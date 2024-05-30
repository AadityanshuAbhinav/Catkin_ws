#! /usr/bin/env python3

import rospy
import numpy as np
import tf
import math 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
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
		#print('x(k-1) = ', x_kminus1, ', x(k) = ', x_k)


	t_k_sec = msg.header.stamp.secs
	t_k_nsec = msg.header.stamp.nsecs
	t_k = t_k_sec + t_k_nsec*10**(-9)

	x_array_k = msg.ranges
	x_array_k_masked = np.ma.masked_equal(x_array_k, 0.0, copy=False)
	x_k = x_array_k_masked.min()
	#print('After masking min dist', x_k)

	theta_k = x_array_k.index(x_k)
	#print('After masking min dist angle', theta_k)


	if abs(x_k-d0) > (dead_zone):
		move.linear.x = 0.0*alpha_x*(x_k-d0)	
	else:
		move.linear.x = 0.0
	

	if theta_k > dead_zone_theta:
		if theta_k <= 180:
			move.angular.z = 0.0*alpha_theta*(theta_k)
		else:
			move.angular.z = 0.0*alpha_theta*(theta_k-359)
	else:
		move.angular.z = 0


	pub.publish(move)
	counter = counter + 1
	
	
def odom_callback(data):
	  global robot_x, robot_y, robot_theta, flag, x_data, y_data, todom_data, theta_data
	  t_o_sec = data.header.stamp.secs
	  t_o_nsec = data.header.stamp.nsecs
	  t_o = t_o_sec + t_o_nsec*10**(-9)
	  
	  robot_x = data.pose.pose.position.x 
	  robot_y = data.pose.pose.position.y
	  print('x: ', robot_x, '    y: ', robot_y)
	  orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
	  euler = tf.transformations.euler_from_quaternion(orientation_list)
	  robot_theta = math.degrees(euler[2]) # in degrees
	  x_data.append(robot_x)
	  y_data.append(robot_y)
	  theta_data.append(robot_theta)
	  todom_data.append(t_o)
	  
	  if (t_o > todom_data[0]+8):
	    x_mat = np.array(x_data)
	    y_mat = np.array(y_data)
	    todom_mat = np.array(todom_data)
	    theta_mat = np.array(theta_data)
	    
	    #scipy.io.savemat('odom_robot4_FORM.mat', dict(t1=todom_mat, x1=x_mat, y1=y_mat, th1=theta_mat))
	  #print("Robot state",robot_theta)


if __name__ == '__main__':
	counter = 0

	rospy.init_node('obstacle_follower_b1')
	sub = rospy.Subscriber('tb3_1/scan', LaserScan, callback)
	pub = rospy.Publisher('tb3_1/cmd_vel', Twist)
	sub2 = rospy.Subscriber('tb3_1/odom', Odometry, odom_callback)
	move = Twist()

	rospy.spin()
