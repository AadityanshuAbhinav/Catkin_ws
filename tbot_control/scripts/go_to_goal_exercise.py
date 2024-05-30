#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
import math 
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# robot pose
robot_x = 0
robot_y = 0
robot_theta = 0 

# Controller gain
K = 0.02

# goal position
goal_x =  0
goal_y =  0

# Commanded velocity 
move = Twist() # defining the way we can allocate the values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0

#############################Edit only this function######
def controller(): 
  global move
  global robot_x, robot_y, robot_theta
  global goal_x, goal_y

  distance_to_goal = ((goal_x-robot_x)**2 + (goal_y-robot_y)**2)**(0.5) 
  if  distance_to_goal < 0.01:
    move.linear.x = 0
    move.angular.z = 0
    return
  
  desired_orientation = math.atan2((goal_y - robot_y),(goal_x - robot_x))
  orientation_error = desired_orientation - robot_theta*(numpy.pi/180)
  
  if orientation_error<-numpy.pi:
    orientation_error+= 2*numpy.pi  
  
  if orientation_error>numpy.pi:
    orientation_error-= 2*numpy.pi 
  
  move.angular.z = K * orientation_error*(180/numpy.pi)
  move.linear.x = 0.05 #Apply constant linear velocity 
  print("Des angle",desired_orientation)
  print("Error", distance_to_goal)
  
def callback(data):
  global robot_x, robot_y, robot_theta, flag
  robot_x = data.pose.pose.position.x 
  robot_y = data.pose.pose.position.y
  orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
  euler = tf.transformations.euler_from_quaternion(orientation_list)
  robot_theta = math.degrees(euler[2]) # in degrees
  controller()
  #print("Robot state",robot_theta)
  
  
##########################################################

rospy.init_node('Go_to_goal')  # Defines a node with name of Go_to_goal
velocity_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_1/odom', Odometry, callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()



'''


'''

