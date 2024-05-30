#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy as np
import scipy.io

theta_in = 0
# robot pose
robot_x = 0
robot_y = 0
robot_theta = 0
ite1 = 0

x_data = []
y_data = []
theta_data = []
todom_data = []
tlidar_data = []
lidar_data = []
P_prev1 = np.array([[0.01,0],[0,0.01]])
P_prev2 = np.array([[0.01,0],[0,0.01]])
M_wF = []
M_F = []
Z2_data = []
Z1_data = []
M1_data = []
M2_data = []
fg = 0
trer_data = []

# Commanded velocity 
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

def controller(min_value_1,min_index_1,min_value_2,min_index_2): 
  global move
  global robot_x, robot_y, robot_theta, x_form_prev_err3 , y_form_prev_err3, ite1,trer_data
  
  ForTheta = 45
  dx  = -d*np.cos(ForTheta*np.pi/180)
  dy  = 3*d*np.sin(ForTheta*np.pi/180)
  l = 0.07
  
  goal_x = 1
  goal_y = 0
  distance_to_goal_x = goal_x-robot_x
  distance_to_goal_y = goal_y-robot_y
  
  distance_to_goal = np.sqrt(distance_to_goal_x**2+distance_to_goal_y**2)
  # Controller gain
  K_tr = 0
  K_for_p = 0.5
  K_for_d = 0
  K_ang = 0.5
  
  x_form_error = dx + min_value_1*np.cos((robot_theta+min_index_1)*np.pi/180)+ min_value_2*np.cos((robot_theta+min_index_2)*np.pi/180)
  y_form_error = dy + min_value_1*np.sin((robot_theta+min_index_1)*np.pi/180)+ min_value_2*np.sin((robot_theta+min_index_2)*np.pi/180)
  
  if(ite1 == 0):
  	x_form_prev_err3 = x_form_error
  	y_form_prev_err3 = y_form_error
  
  x_dot = K_for_p * (x_form_error) + K_tr*distance_to_goal_x + K_for_d * (x_form_error-x_form_prev_err3)/0.2
  y_dot = K_for_p * (y_form_error) + K_tr*distance_to_goal_y + K_for_d * (y_form_error-y_form_prev_err3)/0.2
  
  distance_to_pose = np.sqrt(x_form_error**2+y_form_error**2)
  
  trer_data.append(distance_to_pose)
  
  rob_theta = robot_theta
  if(rob_theta >179):
  	rob_theta = rob_theta-360
  
  if distance_to_pose<0.05:
    move.linear.x = 0
    move.angular.z = 0 #K_ang*((rob_theta-min_index_1)*np.pi/180)
  else:
    move.linear.x = x_dot*np.cos(robot_theta*np.pi/180) + y_dot*np.sin(robot_theta*np.pi/180)
    move.angular.z = (-x_dot*np.sin(robot_theta*np.pi/180) + y_dot*np.cos(robot_theta*np.pi/180))/l
    
  x_form_prev_err3 = x_form_error
  y_form_prev_err3 = y_form_error
  
  print("Tracking Error3", distance_to_pose)
  print("Formation Errorx3", x_form_error)
  print("Formation Errory3", y_form_error)
  print("angular dev3", np.sin(robot_theta+min_index_1))
  print("angular vel3", -K_ang*np.sin(robot_theta+min_index_1))
  print("robot angle3", robot_theta)
  print("lidar angle3", min_index_1)
  print("Robot state3 ", robot_x, " ", robot_y)
  
def LidarAvg(Zcurr,Zprev1,Zprev2,Zprev3):
    M_prev = (Zprev1+Zprev2+Zprev3)/3
    if(Zcurr>3*M_prev):
    	M = M_prev
    else:
    	M = Zcurr
    return M 
#############################Edit only this function######

def lidar_callback(msg):
  global tlidar_data, lidar_data, d, P_prev1, P_prev2, M_prev1, M_prev2, ite1, Z11, Z12, Z21, Z22, M1, Z1prev1, Z1prev2, Z1prev3, M2, Z2prev1, Z2prev2, Z2prev3,trer_data
  #Make sense of the callback data
  #rostopic echo msg
  #0 to 359 aray index values with each value at a degree increment 
  #range should be in meters
  #The readings start from left and go counter clockwise
  t_l_sec = msg.header.stamp.secs
  t_l_nsec = msg.header.stamp.nsecs
  t_l = t_l_sec + t_l_nsec*10**(-9)
  dt = 0.2
  RANGES=list(msg.ranges)
  
  d = 0.4
  
  lidar_data.append(RANGES)
  tlidar_data.append(t_l)
  
  if (t_l > tlidar_data[0]+8):
    lidar_mat = np.array(lidar_data)
    tlidar_mat = np.array(tlidar_data)
    
    scipy.io.savemat('lid_robot3_FORM3.mat', dict(t1lid3=tlidar_mat, lid3=lidar_mat))
    
  RANGES1=list(msg.ranges)
    
  i=0
  while(i<len(RANGES)):
    if(RANGES[i]<=msg.range_min or RANGES[i]>=1.3):
      RANGES[i]=10000
      RANGES1[i]=10000 
    i+=1
    
  min_value_1 = min(RANGES)
  min_index_1 = RANGES.index(min_value_1)
  
  if(min_index_1 > 179):
  	min_index_1 = min_index_1-360
  
  Z1 = [[min_value_1],[min_index_1*np.pi/180]]
  Z1_data.append(Z1)
  Zcurr1 = min_value_1
  
  if(ite1<3):
  	Z1prev1 = Zcurr1
  	Z1prev2 = Zcurr1
  	Z1prev3 = Zcurr1

  M1 = LidarAvg(Zcurr1,Z1prev1,Z1prev2,Z1prev3)
  
  M1_data.append(M1)
  
  Z1prev3 = Z1prev2
  Z1prev2 = Z1prev1
  Z1prev1 = M1
      
  fl = 0
    
  theta = np.ceil(np.arctan(0.1/min_value_1)*180/np.pi)
  thmaxran = np.ceil(np.arctan(0.1/d)*180/np.pi)
  if (theta > thmaxran):
    theta = thmaxran
    
  thmin = min_index_1 - theta
  thmax = min_index_1 + theta
    
  if thmin<0:
    thmin =thmin + 360
    fl = 1
  elif thmax>359:
    thmax = thmax - 360
    fl = 1 
    
  for i in range(len(RANGES)):
    if (fl == 0):
      if(i>thmin and i<thmax):
        RANGES1[i] = 10000
    elif (fl==1):
      if(i>thmin or i<thmax):
        RANGES1[i] = 10000 
    
  min_value_2 = min(RANGES1)
  min_index_2 = RANGES1.index(min_value_2)
  
  Z2 = [[min_value_2],[min_index_2*np.pi/180]]
  Z2_data.append(Z2) 
  
  Zcurr2 = min_value_2
  
  if(ite1<3):
  	Z2prev1 = Zcurr2
  	Z2prev2 = Zcurr2
  	Z2prev3 = Zcurr2

  M2 = LidarAvg(Zcurr2,Z2prev1,Z2prev2,Z2prev3)
  
  M2_data.append(M2)
  
  Z2prev3 = Z2prev2
  Z2prev2 = Z2prev1
  Z2prev1 = M2  
  
  Z1 = np.array(Z1)
  Z2 = np.array(Z2) 
    
  if (t_l > tlidar_data[0]+8):
    lidarM1_mat = np.array(M1_data)
    lidarM2_mat = np.array(M2_data)
    lidarZ1_mat = np.array(Z1_data)
    lidarZ2_mat = np.array(Z2_data)
    
    scipy.io.savemat('Kallid_robot3_FORM3.mat', dict(lid3M11=lidarM1_mat, lid3M21=lidarM2_mat, lid3Z11=lidarZ1_mat, lid3Z21=lidarZ2_mat))
  
  print(" Measurement31 " , Z1," Measurement32 " , Z2)
  print(" Prediction31 " , M1," Prediction32 " , M2)
    
  if(min_value_1 <= msg.range_max):
    controller(M1,min_index_1,M2,min_index_2)
  else:
    move.linear.x = 0
    move.linear.y = 0
    move.linear.z = 0
    move.angular.x = 0
    move.angular.y = 0
    move.angular.z = 0
  
  if (t_l > tlidar_data[0]+8):
  	trer3_mat = np.array(trer_data)
  	
  	scipy.io.savemat('TRER_robot3_FORM3.mat', dict(trer3=trer3_mat))
  
  ite1 = ite1+1
#############################Edit only this function######
  
def odom_callback(data):
  global robot_x, robot_y, robot_theta, flag, x_data, y_data, todom_data, theta_data
  t_o_sec = data.header.stamp.secs
  t_o_nsec = data.header.stamp.nsecs
  t_o = t_o_sec + t_o_nsec*10**(-9)
  
  robot_x = 1000*data.pose.pose.position.x 
  robot_y = 1000*data.pose.pose.position.y
  orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(orientation_list)
  robot_theta = math.degrees(euler[2]) # in degrees
  
  if (robot_theta<0):
    robot_theta = robot_theta + 360
  
  x_data.append(robot_x)
  y_data.append(robot_y)
  theta_data.append(robot_theta)
  todom_data.append(t_o)
  
  if (t_o > todom_data[0]+8):
    x_mat = np.array(x_data)
    y_mat = np.array(y_data)
    todom_mat = np.array(todom_data)
    theta_mat = np.array(theta_data)
    
    scipy.io.savemat('odom_robot3_FORM3.mat', dict(t31=todom_mat, x31=x_mat, y31=y_mat, th31=theta_mat))
  
  
##########################################################

rospy.init_node('Form33')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_1/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_1/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


