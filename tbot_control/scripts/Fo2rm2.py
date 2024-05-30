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
ite = 0 

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
Z_data = []
M_data = []
trer_data = []

# Commanded velocity 
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

  
def controller(min_value_1,min_index_1): 
  global move
  global robot_x, robot_y, robot_theta ,x_form_prev_err ,y_form_prev_err,ite,trer_data
  
  dx  = -0.4
  dy  = 0
  l = 0.07
  
  goal_x = 2
  goal_y = 0
  distance_to_goal_x = goal_x-robot_x
  distance_to_goal_y = goal_y-robot_y
  
  distance_to_goal = np.sqrt(distance_to_goal_x**2+distance_to_goal_y**2)
  # Controller gain
  K_tr = 0
  K_for_p = 0.5
  K_for_d = 0
  K_ang = 0.5
  
  x_form_error = dx + min_value_1*np.cos((robot_theta+min_index_1)*np.pi/180)
  y_form_error = dy + min_value_1*np.sin((robot_theta+min_index_1)*np.pi/180)
  
  if(ite == 0):
  	x_form_prev_err = x_form_error
  	y_form_prev_err = y_form_error
  
  x_dot = K_for_p * (x_form_error) + K_tr*distance_to_goal_x + K_for_d * (x_form_error-x_form_prev_err)/0.2
  y_dot = K_for_p * (y_form_error) + K_tr*distance_to_goal_y + K_for_d * (y_form_error-y_form_prev_err)/0.2
  
  distance_to_pose = np.sqrt(x_form_error**2+y_form_error**2)
  
  trer_data.append(distance_to_pose)
  
  rob_theta = robot_theta
  if(rob_theta >179):
  	rob_theta = rob_theta-360
  	
  	
  if distance_to_pose < 0.05:
    move.linear.x = 0
    move.angular.z = -K_ang*((rob_theta-min_index_1)*np.pi/180)
  else:
    move.linear.x = x_dot*np.cos(robot_theta*np.pi/180) + y_dot*np.sin(robot_theta*np.pi/180)
    move.angular.z = (-x_dot*np.sin(robot_theta*np.pi/180) + y_dot*np.cos(robot_theta*np.pi/180))/l
  
  x_form_prev_err = x_form_error
  y_form_prev_err = y_form_error
  
  print("Tracking Error", distance_to_pose)
  print("Formation Errorx", x_form_error)
  print("Formation Errory", y_form_error)
  print("angular velo", (-x_dot*np.sin(robot_theta*np.pi/180) + y_dot*np.cos(robot_theta*np.pi/180))/l)
  print("robot angle", robot_theta)
  print("lidar angle", min_index_1)

def LidarTustin(M_prev,Z1,Z2,dt):
    w_o = 2
    di1 = Z1[0,0]
    di2 = Z2[0,0]
    phii1 = Z1[1,0]
    phii2 = Z2[1,0]
    
    do1 = M_prev[0,0]
    phio1 = M_prev[1,0]
    
    do2 = (di1+di2)*(w_o*dt/(2+w_o*dt)) - ((w_o*dt-2)/(w_o*dt+2))*do1
    phio2 = (phii1+phii2)*(w_o*dt/(2+w_o*dt)) - ((w_o*dt-2)/(w_o*dt+2))*phio1
    
    M = np.array([[do2],[phio2]])
    return M  
    
def LidarAvg(Zcurr,Zprev1,Zprev2,Zprev3):
    M_prev = (Zprev1+Zprev2+Zprev3)/3
    if(Zcurr>3*M_prev):
    	M = M_prev
    else:
    	M = Zcurr
    return M   
#############################Edit only this function######

def lidar_callback(msg):
  global tlidar_data, lidar_data, d, P_prev1, P_prev2, M_prev1, M_prev2, ite, Z11, Z12, Z21, Z22,M, Zprev1, Zprev2, Zprev3,trer_data
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
  
  d = 0.5
  
  lidar_data.append(RANGES)
  tlidar_data.append(t_l)
  
  if (t_l > tlidar_data[0]+8):
    lidar_mat = np.array(lidar_data)
    tlidar_mat = np.array(tlidar_data)
    
    scipy.io.savemat('lid_robot2_FORM2.mat', dict(t1lid=tlidar_mat, lid=lidar_mat))
    
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
  
  Z = [[min_value_1],[min_index_1*np.pi/180]]
  Z_data.append(Z)
  Zcurr = min_value_1
  
  if(ite<3):
  	Zprev1 = Zcurr
  	Zprev2 = Zcurr
  	Zprev3 = Zcurr

  M = LidarAvg(Zcurr,Zprev1,Zprev2,Zprev3)
  
  M_data.append(M)
  
  if (t_l > tlidar_data[0]+8):
    lidarM_mat = np.array(M_data)
    lidarZ_mat = np.array(Z_data)
    
    scipy.io.savemat('Kallid_robot2_FORM2.mat', dict(lidM=lidarM_mat, lidZ=lidarZ_mat))
  
  Zprev3 = Zprev2
  Zprev2 = Zprev1
  Zprev1 = M  
  
  """  
  fl = 0
    
  theta = np.ceil(np.arctan(0.1/min_value_1)*180/np.pi)
  thmaxran = np.ceil(np.arctan(0.1/d)*180/np.pi)
  if (theta > thmaxran):
    theta = thmaxran
    
  thmin = min_index_1 - theta
  thmax = min_index_1 + theta
  
  Z11 = [[min_value_1],[min_index_1*np.pi/180]] 
  
  Z1_data.append(Z11)
    
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
  
  Z21 = [[min_value_2],[min_index_2*np.pi/180]]
  Z2_data.append(Z21) 
  
  Z11 = np.array(Z11)
  Z21 = np.array(Z21) 
  
  if(ite == 0):
    M_prev1 = Z11
    M_prev2 = Z21
    Z12 = Z11
    Z22 = Z21
  
  ite = ite+1
  
  #V_prev =np.array([0,0]) 
  M_prev1 = LidarTustin(M_prev1,Z11,Z12,dt)
  M_prev2 = LidarTustin(M_prev2,Z21,Z22,dt)
  
  M1_data.append(list(M_prev1))
  M2_data.append(list(M_prev2))
  
  Z12 = Z11
  Z22 = Z21
    
  if (t_l > tlidar_data[0]+8):
    lidarM1_mat = np.array(M1_data)
    lidarM2_mat = np.array(M2_data)
    lidarZ1_mat = np.array(Z1_data)
    lidarZ2_mat = np.array(Z2_data)
    
    scipy.io.savemat('Kallid_robot2_FORM3.mat', dict(lidM11=lidarM1_mat, lidM21=lidarM2_mat, lidZ11=lidarZ1_mat, lidZ21=lidarZ2_mat))
  """
  print(" Measurement1 " , Zcurr)
  print(" Prediction1 " , M)
    
  if(min_value_1 <= msg.range_max):
    controller(M,min_index_1)
  else:
    move.linear.x = 0
    move.linear.y = 0
    move.linear.z = 0
    move.angular.x = 0
    move.angular.y = 0
    move.angular.z = 0
  
  if (t_l > tlidar_data[0]+8):
  	trer2_mat = np.array(trer_data)
  	
  	scipy.io.savemat('TRER_robot2_FORM2.mat', dict(trer2=trer2_mat))
  	
  ite = ite+1        
#############################Edit only this function######
  
def odom_callback(data):
  global robot_x, robot_y, robot_theta, flag, x_data, y_data, todom_data, theta_data
  t_o_sec = data.header.stamp.secs
  t_o_nsec = data.header.stamp.nsecs
  t_o = t_o_sec + t_o_nsec*10**(-9)
  
  robot_x = data.pose.pose.position.x 
  robot_y = data.pose.pose.position.y
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
    
    scipy.io.savemat('odom_robot2_FORM2.mat', dict(t1=todom_mat, x1=x_mat, y1=y_mat, th1=theta_mat))
  #print("Robot state",robot_theta)
  
  
##########################################################

rospy.init_node('Form2')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_0/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_0/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


