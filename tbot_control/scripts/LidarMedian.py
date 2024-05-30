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

Z2_data = []
Z1_data = []

fg = 0

# Commanded velocity 
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0
##########################################################

def controller():
    global goal_x,goal_y,V_prev,fg
    goal_x = 0.6
    goal_y = 0
    
    distance_to_goal_x = goal_x-robot_x
    distance_to_goal_y = goal_y-robot_y
    distance_to_goal = np.sqrt(distance_to_goal_x**2+distance_to_goal_y**2)
    
    if(fg==0):
    	move.linear.x = 0.5*distance_to_goal
    	move.angular.z = 0
    
    if(distance_to_goal<0.05):
        move.linear.x = 0
        move.angular.z = 0
        fg=1

##########################################################

    
def LidarMed(Z1,Z2,Z3):
    if(Z1[1,0]>np.pi):
    	Z1[1,0] = Z1[1,0]-2*np.pi
    if(Z2[1,0]>np.pi):
    	Z2[1,0] = Z2[1,0]-2*np.pi
    if(Z3[1,0]>np.pi):
    	Z3[1,0] = Z3[1,0]-2*np.pi

    dd = np.sort(np.array([Z1[0,0],Z2[0,0],Z3[0,0]]))
    phi = np.sort(np.array([Z1[1,0],Z2[1,0],Z3[1,0]]))
    
    for i in range(len(phi)):
    	if(phi[i]<0):
    	   phi[i] = phi[i]+2*np.pi
    	   
    M = np.array([[dd[1]],[phi[1]]])  
    return M
  
#############################Edit only this function######


          
          
def lidar_callback(msg):
  global tlidar_data, lidar_data, d, Z11,Z12,Z13,Z21,Z22,Z23,ite
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
    
    scipy.io.savemat('lidMed_robot4_FORM3.mat', dict(t11lid=tlidar_mat, lid=lidar_mat))
    
  RANGES1=list(msg.ranges)
    
  i=0
  while(i<len(RANGES)):
    if(RANGES[i]<=msg.range_min or RANGES[i]>=1.3):
      RANGES[i]=10000
      RANGES1[i]=10000 
    i+=1
    
  min_value_1 = min(RANGES)
  min_index_1 = RANGES.index(min_value_1)
  
  Z11 = [[min_value_1],[min_index_1*np.pi/180]] 
  Z11 = np.array(Z11)
  
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
  
  Z21 = [[min_value_2],[min_index_2*np.pi/180]]
  Z21 = np.array(Z21)
  
  if(ite == 0):
    Z12 = Z11
    Z13 = Z11
    Z22 = Z21
    Z23 = Z21
    
  if(ite == 1):
    Z13 = Z12
    Z23 = Z22
  
  if Z11[1,0]>1.3 or Z11[1,0]<0.1:
  	Z11 = LidarMed(Z11,Z12,Z13)
  if Z21[1,0]>1.3 or Z11[1,0]<0.1: 
  	Z21 = LidarMed(Z21,Z22,Z23)
  
  Z1_data.append(list(Z11))
  Z2_data.append(list(Z21))
  
  ite = ite+1
  
  Z13 = Z12
  Z23 = Z22
  
  Z12 = Z11
  Z22 = Z21
    
  if (t_l > tlidar_data[0]+8):
    lidarZ1_mat = np.array(Z1_data)
    lidarZ2_mat = np.array(Z2_data)
    
    scipy.io.savemat('KallidMed_robot4_FORM3.mat', dict(lidZ11=lidarZ1_mat, lidZ21=lidarZ2_mat))
  
  print(ite, " Measurement1 " , Z12," Measurement2 " , Z22)
  
  controller()
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
    
    scipy.io.savemat('odomMed_robot4_FORM3.mat', dict(t1=todom_mat, x1=x_mat, y1=y_mat, th1=theta_mat))
  #print("Robot state",robot_theta)
  
  
##########################################################

rospy.init_node('Form32')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_0/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_0/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


