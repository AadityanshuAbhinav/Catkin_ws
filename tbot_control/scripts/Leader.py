#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy

# robot pose
robot_x = 0
robot_y = 0
robot_theta = 0 


# Commanded velocity 
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

"""
def controller(goal_x,goal_y): 
  global move
  global robot_x, robot_y, robot_theta
  
  # Controller gain
  K1 = 0.2
  K2 = 0.02

  distance_to_goal = ((goal_x-robot_x)**2 + (goal_y-robot_y)**2)**(0.5) 
  if  distance_to_goal < 0.05:
    move.linear.x = 0
    move.angular.z = 0
    return
  
  desired_orientation = math.atan2((goal_y - robot_y),(goal_x - robot_x))
  orientation_error = desired_orientation - robot_theta*(numpy.pi/180)
  
  orientation_error%=(2*numpy.pi)  
  
  if orientation_error>numpy.pi:
    orientation_error-= 2*numpy.pi 
  
  move.angular.z = K2 * orientation_error*(180/numpy.pi)
  move.linear.x = K1 * distance_to_goal 
  print("Des angle",desired_orientation)
  print("Error", distance_to_goal)
"""

def controller(v, omega):
  move.linear.x = v
  move.angular.z = omega

#############################Edit only this function######
def lidar_callback(msg):
  #Make sense of the callback data
  #rostopic echo msg
  #0 to 359 aray index values with each value at a degree increment 
  #range should be in meters
  #The readings start from left and go counter clockwise
  RANGES=list(msg.ranges)
    
  dist = 0.5
    
  i=0
  while(i<len(RANGES)):
    if(RANGES[i]<=msg.range_min or RANGES[i]>=msg.range_max):
      RANGES[i]=10000
    i+=1
    
  goal_x = robot_x + dist*numpy.cos((robot_theta)*numpy.pi/180)
  goal_y = robot_y + dist*numpy.sin((robot_theta)*numpy.pi/180)
  
  controller(0.05 , 0)

#############################Edit only this function######
  
def odom_callback(data):
  global robot_x, robot_y, robot_theta, flag
  robot_x = data.pose.pose.position.x 
  robot_y = data.pose.pose.position.y
  orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(orientation_list)
  robot_theta = math.degrees(euler[2]) # in degrees
  #print("Robot state",robot_theta)
  
  
##########################################################

rospy.init_node('Leader')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_0/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_0/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


