#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import os
import scipy.io
import math


t0 = 0

d0 = 0.2
dead_zone = 0.005
alpha_x = 1.5
beta_m_x = 0.1

dead_zone_theta = 2
alpha_theta = 0.07
beta_m_theta = 0.15

T = 5
A = 0.4*2*3.14/(T**2)

num_of_iters = 31
iterations = 0


class Server:

	def __init__(self, iterations):
		self.lidar_dist = None
		self.lidar_theta = None
		self.odom_vel = None
		self.odom_ang_vel = None
		self.angle_z_axis = None
		self.iter = 0
		self.odom_iter = 0
		self.tinit = 0

		#arrays for lidar (5 Hz)
		self.t_mat = np.empty([1, num_of_iters])
		self.x_mat = np.empty([1, num_of_iters])
		self.theta_mat = np.empty([1, num_of_iters])

		#arrays for odometry (29 Hz)
		self.odom_t_mat = np.zeros([1, num_of_iters*6])
		self.odom_v_mat = np.zeros([1, num_of_iters*6])
		self.odom_w_mat = np.zeros([1, num_of_iters*6])
		self.odom_vd_mat = np.zeros([1, num_of_iters*6])


	def lidar_callback(self, msg):
		t_k_sec = msg.header.stamp.secs
		t_k_nsec = msg.header.stamp.nsecs
		t_k = t_k_sec + t_k_nsec*10**(-9)


		x_array_k = msg.ranges

		x_array_0to44 = x_array_k[0:30]
		x_array_0to44_masked = np.ma.masked_equal(x_array_0to44, 0.0, copy=False)
		x_k_min1 = x_array_0to44_masked.min()
		theta_k_1  = x_array_0to44.index(x_k_min1)

		x_array_315to359 = x_array_k[330:360]
		x_array_315to359_masked = np.ma.masked_equal(x_array_315to359, 0.0, copy=False)
		x_k_min2 = x_array_315to359_masked.min()
		theta_k_2 = x_array_315to359.index(x_k_min2)



		#x_array_k_masked = np.ma.masked_equal(x_array_k, 0.0, copy=False)
		#x_k = x_array_k_masked.min()

		#theta_k = x_array_k.index(x_k)




		if x_k_min1 <= x_k_min2:
			x_k = x_k_min1
			theta_k = theta_k_1
		else:
			x_k = x_k_min2
			theta_k = theta_k_2+330


		#print('x_k: ', x_k, ' theta_k: ', theta_k)


		if self.iter < num_of_iters-1:
			self.t_mat[0,self.iter] = t_k
			self.x_mat[0,self.iter] = x_k
			self.theta_mat[0,self.iter] =  theta_k

			self.iter = self.iter + 1
		else:
			print('xmat: ', self.x_mat)
			print('tmat: ', self.t_mat)
			print('thetamat: ', self.theta_mat)
			move.linear.x = 0
			move.angular.z = 0
			pub.publish(move)

			scipy.io.savemat('Xresp4_bot2_nodsr.mat', dict(t1=self.t_mat, x1=self.x_mat, th1=self.theta_mat, O_t1=self.odom_t_mat, O_v1 = self.odom_v_mat, O_w1=self.odom_w_mat))

			os._exit(0)



		if abs(x_k-d0) > (dead_zone):
			move.linear.x = alpha_x*(x_k-d0) #+ beta_m_x*self.odom_vel
			#if self.iter < 1:
			#	move.linear.x = alpha_x*(x_k-d0)
			#else:
			#	dt = self.t_mat[0,self.iter]-self.t_mat[0,self.iter-1]
			#	x_kminus1 = self.x_mat[0,self.iter-1]

			#	move.linear.x = self.odom_vel - beta_m_x*(-x_k+x_kminus1)/dt - alpha_x*beta_m_x*(-x_k+d0)
		else:
			move.linear.x = 0.0



		#if theta_k > dead_zone_theta:
			#if theta_k <= 180:
				#move.angular.z = alpha_theta*(theta_k) + beta_m_theta*self.odom_ang_vel
			#else:
				#move.angular.z = alpha_theta*(theta_k-359) + beta_m_theta*self.odom_ang_vel
		#else:
			#move.angular.z = 0

		if theta_k <= 180:
			if theta_k > dead_zone_theta:
				move.angular.z = alpha_theta*theta_k #+ beta_m_theta*self.odom_ang_vel

				#if self.iter < 1:
				#	move.angular.z  = 0*alpha_theta*theta_k
				#else:
				#	d_theta_k = theta_k
				#	d_theta_kminus1 = self.theta_mat[0,self.iter-1]

				#	dt = self.t_mat[0, self.iter] - self.t_mat[0, self.iter-1]

				#	if d_theta_kminus1 > 180:
				#		d_theta_kminus1 = -(360 - d_theta_k)

				#	move.angular.z = 0*( self.odom_ang_vel - beta_m_theta*(-1)*(d_theta_k-d_theta_kminus1)/dt - alpha_theta*beta_m_theta*(-1)*d_theta_k )

			else:
				move.angular.z = 0

		else:
			if 360-theta_k > dead_zone_theta:
				move.angular.z = alpha_theta*(theta_k-359) #+ beta_m_theta*self.odom_ang_vel
				#if self.iter < 1:
				#	move.angular.z = 0*alpha_theta*(theta_k-360)
				#else:
				#	d_theta_k = 360-theta_k
				#	d_theta_kminus1 = self.theta_mat[0, self.iter-1]

				#	dt = self.t_mat[0,self.iter] - self.t_mat[0, self.iter-1]

				#	if d_theta_kminus1 <= 180:
				#		d_theta_kminus1 = -(360 - d_theta_k)

				#	move.angular.z = 0*( self.odom_ang_vel - beta_m_theta*(d_theta_k - d_theta_kminus1)/dt - alpha_theta*beta_m_theta*d_theta_k )


			else:
				move.angular.z = 0

		#move.linear.x = 0.157
		#if self.iter<=5:
		#	move.angular.z = 0*0.625
		#	move.linear.x = 0.0
		#else:
		#	move.angular.z = -0.625*0
		#	move.linear.x = 0.20

		#print('angle z: ', self.angle_z_axis)
		#print('move x: ', move.linear.x, ', move z: ', move.angular.z)


		pub.publish(move)

	def odom_callback(self, msg):

		self.odom_vel = msg.twist.twist.linear.x
		self.odom_ang_vel = msg.twist.twist.angular.z


		if self.odom_iter <= num_of_iters*6-1:
			o_tk = msg.header.stamp.secs
			o_tk_n = msg.header.stamp.nsecs
			self.odom_t_mat[0, self.odom_iter] = o_tk + o_tk_n*10**(-9)
			self.odom_v_mat[0, self.odom_iter] = self.odom_vel
			self.odom_w_mat[0, self.odom_iter] = self.odom_ang_vel

			if self.odom_iter == 0:
				self.tinit = self.odom_t_mat[0,self.odom_iter]
				v_d = 0.1 #0
			else:
				delta_t = self.odom_t_mat[0, self.odom_iter] -self.tinit
				v_d = 0.1 #A*(1-math.cos( 2*3.14*delta_t/T ))/( 2*3.14/T )


			self.odom_vd_mat[0, self.odom_iter] = v_d

			self.odom_iter = self.odom_iter + 1

			#if (self.odom_t_mat[0,self.odom_iter-1]-t0) >= 1:
				#print('t: ', self.odom_t_mat[0,self.odom_iter-1]-self.odom_t_mat[0,0])

			move.linear.x = v_d #0.20
			pub.publish(move)

		else:
			move.linear.x = 0
			pub.publish(move)
			scipy.io.savemat('Step_b2_red_delay_v2.mat', dict(O_t1=self.odom_t_mat, O_v1=self.odom_v_mat, t_init=self.tinit, O_vd1 = self.odom_vd_mat))

			os._exit(0)



		#bot_orientation = msg.pose.pose.orientation
		#q_w = bot_orientation.w
		#q_x = bot_orientation.x
		#q_y = bot_orientation.y
		#q_z = bot_orientation.z

		#siny_cosp = 2*(q_w*q_z + q_x*q_y)
		#cosy_cosp = 1 - 2*(q_y*q_y + q_z*q_z)
		#self.angle_z_axis = math.atan2(siny_cosp, cosy_cosp)



if __name__ == '__main__':
	rospy.init_node('obstacle_follower_b2')
	move = Twist()

	t0 = time.time()
	#print('t0: ', t0)

	server = Server(iterations)
	#rospy.Subscriber('tb3_0/scan', LaserScan, server.lidar_callback)
	rospy.Subscriber('tb3_0/odom', Odometry, server.odom_callback)

	pub = rospy.Publisher('tb3_0/cmd_vel', Twist,queue_size=10)


	rospy.spin()
