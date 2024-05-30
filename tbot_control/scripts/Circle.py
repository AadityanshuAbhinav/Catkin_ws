#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy
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


def controller(): 
  global move
  
  move.angular.z = 0
  move.linear.x = 0

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
  controller()
  
  lidar_data.append(RANGES)
  tlidar_data.append(t_l)
  
  if (t_l > tlidar_data[0]+8):
    lidar_mat = numpy.array(lidar_data)
    tlidar_mat = numpy.array(tlidar_data)
    
    scipy.io.savemat('resplid_robot1.mat', dict(t1lid=tlidar_mat, lid=lidar_mat))
          
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
    x_mat = numpy.array(x_data)
    y_mat = numpy.array(y_data)
    todom_mat = numpy.array(todom_data)
    theta_mat = numpy.array(theta_data)
    
    scipy.io.savemat('resp_robot1.mat', dict(t1=todom_mat, x1=x_mat, y1=y_mat, th1=theta_mat))
  #print("Robot state",robot_theta)
  
  
##########################################################

rospy.init_node('Circle')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_0/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_0/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


