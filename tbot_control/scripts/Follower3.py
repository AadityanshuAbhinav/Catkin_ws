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

#############################Edit only this function######
def lidar_callback(msg):
    #Make sense of the callback data
    #rostopic echo msg
    #0 to 359 aray index values with each value at a degree increment 
    #range should be in meters
    #The readings start from left and go counter clockwise
    RANGES=list(msg.ranges)
    
    d = 0.5
    
    i=0
    while(i<len(RANGES)):
        if(RANGES[i]<=msg.range_min or RANGES[i]>=msg.range_max):
            RANGES[i]=10000
        i+=1
    
    for i in range(91):
	 left.append(RANGES[i])
	 right.append(RANGES[i+270])
    
    RANGES1 = [right,left]
    
    
    min_value = min(RANGES1)
    min_index = RANGES1.index(min_value)
    
    if(min_index<90):
        min_index = min_index+270
    else:
        min_index = min_index-90
    
    goal_x = robot_x + (min_value-d)*numpy.cos((min_index+robot_theta)*numpy.pi/180)
    goal_y = robot_y + (min_value-d)*numpy.sin((min_index+robot_theta)*numpy.pi/180)
    
    if(min_value <= msg.range_max):
    	controller(goal_x,goal_y)
    else:
    	move.linear.x = 0
    	move.linear.y = 0
    	move.linear.z = 0
    	move.angular.x = 0
    	move.angular.y = 0
    	move.angular.z = 0
    """
    #Get range measurements from the front left side of the robot [from 0degrees to 90degrees]
    left=list(RANGES[0:91])

    #Get range measurements from the front right side of the robot [from 270degrees to 359degrees]
    right=list(RANGES[270:])

    
    ##Replace zero readings with some large number
    i=0
    while(i<len(left)):
        if(left[i]<=msg.range_min or ):
            left[i]=10000
        i+=1
        
    i=0
    while(i<len(right)):
        if(right[i]<=msg.range_min):
            right[i]=10000
        i+=1

    
    #Find the minimum range measurement from both sides
    min_range_left=min(left)
    min_range_right=min(right)

    
    if min_range_left > 0.4 and min_range_right > 0.4:
        print("Drive straight")
        controller(0.05,0.0)
        return
	
    if min_range_left < 0.4 and min_range_right > 0.4:
        print("Turn right")
        controller(0.05, -0.225)
        return
        
    if min_range_left > 0.4 and min_range_right < 0.4:
        print("Turn left")
        controller(0.05, 0.225)
        return
        
    if min_range_left < 0.2 and min_range_right < 0.2:
        print("Stop robot")
        controller(0.0, 0.0)
        return  """
        
    
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

rospy.init_node('Follower3')  # Defines a node with name of Follower
velocity_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('tb3_1/odom', Odometry, odom_callback)
lidar_subscriber = rospy.Subscriber('tb3_1/scan', LaserScan, lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
  velocity_pub.publish(move)
  rate.sleep()


