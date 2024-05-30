#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy as np
import scipy.io

# robot pose
robot_x = 0
robot_y = 0
robot_theta = 0 

x_data = []
y_data = []
theta_data = []
todom_data = []
tlidar_data = []
lidar_data = []

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
  global robot_x, robot_y, robot_theta
  
  d  = 0.5
  l = 0.05
  
  goal_x = 2
  goal_y = 0
  distance_to_goal_x = np.abs(robot_x-goal_x)
  distance_to_goal_y = np.abs(robot_y-goal_y)
  
  distance_to_goal = np.sqrt(distance_to_goal_x**2+distance_to_goal_y**2)
  # Controller gain
  K_tr = 0
  K_for = 0.1
  
  x_dot = K_for * (-d*np.cos(30*np.pi/180)+min_value_1*np.cos(min_index_1*np.pi/180)+ min_value_2*np.cos(min_index_2*np.pi/180)) + K_tr*distance_to_goal_x
  y_dot = K_for * (3*d*np.sin(30*np.pi/180)+min_value_1*np.sin(min_index_1*np.pi/180)+ min_value_2*np.sin(min_index_2*np.pi/180)) + K_tr*distance_to_goal_y
  
  move.linear.x = x_dot*np.cos(robot_theta*np.pi/180) + y_dot*np.sin(robot_theta*np.pi/180)
  move.angular.z = (-x_dot*np.sin(robot_theta*np.pi/180) + y_dot*np.cos(robot_theta*np.pi/180))/l
  
  print("Tracking Error", distance_to_goal)
  print("Formation Errorx", -d*np.cos(30*np.pi/180)+min_value_1*np.cos(min_index_1*np.pi/180)+ min_value_2*np.cos(min_index_2*np.pi/180))
  print("Formation Errory", 3*d*np.sin(30*np.pi/180)+min_value_1*np.sin(min_index_1*np.pi/180)+ min_value_2*np.sin(min_index_2*np.pi/180))
  
  
#############################Edit only this function######

def lidar_callback(msg):
  global tlidar_data, lidar_data
  #Make sense of the callback data
  #rostopic echo msg
  #0 to 359 aray index values with each value at a degree increment 
  #range should be in meters
  #The readings start from left and go counter clockwise
  t_l_sec = msg.header.stamp.secs
  t_l_nsec = msg.header.stamp.nsecs
  t_l = t_l_sec + t_l_nsec*10**(-9)
  
  RANGES=list(msg.ranges)
  
  lidar_data.append(RANGES)
  tlidar_data.append(t_l)
  
  if (t_l > tlidar_data[0]+8):
    lidar_mat = np.array(lidar_data)
    tlidar_mat = np.array(tlidar_data)
    
    scipy.io.savemat('lid_robot3_FORM.mat', dict(t1lid=tlidar_mat, lid=lidar_mat))
    
  RANGES1=list(msg.ranges)
    
  i=0
  while(i<len(RANGES)):
    if(RANGES[i]<=msg.range_min or RANGES[i]>=1.3):
      RANGES[i]=10000
      RANGES1[i]=10000 
    i+=1
    
  min_value_1 = min(RANGES)
  min_index_1 = RANGES.index(min_value_1)
    
  fl = 0
    
  theta = np.ceil(np.arctan(0.05/min_value_1)*180/np.pi)
  thmaxran = np.ceil(np.arctan(0.05/d)*180/np.pi)
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
    
  if(min_value_1 <= msg.range_max or min_value_2 <= msg.range_max):
    controller(min_value_1,min_index_1,min_value_2,min_index_2)
  else:
    move.linear.x = 0
    move.linear.y = 0
    move.linear.z = 0
    move.angular.x = 0
    move.angular.y = 0
    move.angular.z = 0
          
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
    
    scipy.io.savemat('odom_robot3_FORM.mat', dict(t1=todom_mat, x1=x_mat, y1=y_mat, th1=theta_mat))
  #print("Robot state",robot_theta)
  
  
##########################################################

rospy.init_node('Form3')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_1/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_1/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


