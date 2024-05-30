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
V_prev =np.array([0,0]) 

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
d1_data = []
d2_data = []
fg = 0
t_l_1 = 0

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
    	V_prev[0] = 0.5*distance_to_goal
    	V_prev[1] = 0
    else:
    	move.linear.x = 0
    	move.angular.z = 0
    	V_prev[0] = 0
    	V_prev[1] = 0
        
    if(distance_to_goal<0.05):
        move.linear.x = 0
        move.angular.z = 0
        V_prev[0] = 0
        V_prev[1] = 0
        fg=1

##########################################################
def LidarMin2(P_prev1,P_prev2,M_prev1,M_prev2,V_prev,Z1,Z2,theta,dt):
    k = 100
    k2 = 0.001
    d_11 = M_prev1[0,0]
    phi_11 = M_prev1[1,0]
    
    d_12 = M_prev2[0,0]
    phi_12 = M_prev2[1,0]
    
    #print(d_11, " ", d_12)
    v = V_prev[0]
    w = V_prev[1]
    
    d2_hat1 = np.sqrt(d_11**2+(v*dt)**2 -2*d_11*v*dt*np.cos(phi_11-w*dt*0.5))
    phi2_hat1 = np.arctan2(d_11*np.sin(phi_11+theta)-v*dt*np.sin(theta+w*dt*0.5),d_11*np.cos(phi_11+theta)-v*dt*np.cos(theta+w*dt*0.5)) - (theta+w*dt)
    if(phi2_hat1<0):
    	phi2_hat1=phi2_hat1+2*np.pi
    
    d2_hat2 = np.sqrt(d_12**2+(v*dt)**2 -2*d_12*v*dt*np.cos(phi_12-w*dt*0.5))
    phi2_hat2 = np.arctan2(d_12*np.sin(phi_12+theta)-v*dt*np.sin(theta+w*dt*0.5),d_12*np.cos(phi_12+theta)-v*dt*np.cos(theta+w*dt*0.5)) - (theta+w*dt)
    if(phi2_hat2<0):
    	phi2_hat2=phi2_hat2+2*np.pi
    	
    a1 = d_11*np.sin(phi_11+theta)-v*dt*np.sin(theta+w*dt*0.5)
    b1 = d_11*np.cos(phi_11+theta)-v*dt*np.cos(theta+w*dt*0.5)
    
    a2 = d_12*np.sin(phi_12+theta)-v*dt*np.sin(theta+w*dt*0.5)
    b2 = d_12*np.cos(phi_12+theta)-v*dt*np.cos(theta+w*dt*0.5)
    
    F_p1 = np.array([[(d_11-v*dt*np.cos(phi_11-w*dt*0.5))/d2_hat1,(2*d_11*v*np.sin(phi_11-w*dt*0.5))/d2_hat1],[(-a1*np.cos(theta+phi_11)+b1*np.sin(theta+phi_11))/(a1**2+b1**2),(d_11*b1*np.cos(theta+phi_11)+d_11*a1*np.sin(theta+phi_11))/(a1**2+b1**2)]])
    F_del1 = np.array([[(v*dt**2-d_11*dt*np.cos(phi_11-w*dt*0.5))/d2_hat1,(-d_11*v*dt**2*np.sin(phi_11-w*dt*0.5))/d2_hat1],[(-a1*dt*np.sin(theta+w*dt*0.5)+b1*dt*np.cos(theta+w*dt*0.5))/(a1**2+b1**2),(-v*dt*b1*np.cos(theta+w*dt*0.5)-dt*a1*v*np.sin(theta+w*dt*0.5))/(a1**2+b1**2)]])
    F_p2 = np.array([[(d_12-v*dt*np.cos(phi_12-w*dt*0.5))/d2_hat2,(2*d_12*v*np.sin(phi_12-w*dt*0.5))/d2_hat2],[(-a2*np.cos(theta+phi_12)+b2*np.sin(theta+phi_12))/(a2**2+b2**2),(d_12*b2*np.cos(theta+phi_12)+d_12*a2*np.sin(theta+phi_12))/(a2**2+b2**2)]])
    F_del2 = np.array([[(v*dt**2-d_12*dt*np.cos(phi_12-w*dt*0.5))/d2_hat2,(-d_12*v*dt**2*np.sin(phi_12-w*dt*0.5))/d2_hat2],[(-a2*dt*np.sin(theta+w*dt*0.5)+b2*dt*np.cos(theta+w*dt*0.5))/(a2**2+b2**2),(-v*dt*b2*np.cos(theta+w*dt*0.5)-dt*a2*v*np.sin(theta+w*dt*0.5))/(a2**2+b2**2)]])
    
    Q = k*np.array([[np.abs(v),0],[0,np.abs(w)]])
    H = np.array([[1,0],[0,1]])
    R = np.array([[0.1,0],[0,0.1]])
    
    P_hat1 = np.dot(F_p1,np.dot(P_prev1,F_p1.transpose())) + np.dot(F_del1,np.dot(Q,F_del1.transpose()))
    #print(P_hat1)
    Sig_in1 = np.dot(H,np.dot(P_hat1,H.transpose())) + R
    
    P_hat2 = np.dot(F_p2,np.dot(P_prev2,F_p2.transpose())) + np.dot(F_del2,np.dot(Q,F_del2.transpose()))
    Sig_in2 = np.dot(H,np.dot(P_hat2,H.transpose())) + R
    
    K1 = np.dot(P_hat1,np.dot(H,np.linalg.inv(Sig_in1)))
    M_hat1 = np.array([[d2_hat1],[phi2_hat1]])
    mu1 = Z1 - M_hat1
    
    K2 = np.dot(P_hat2,np.dot(H,np.linalg.inv(Sig_in2)))
    M_hat2 = np.array([[d2_hat2],[phi2_hat2]])
    mu2 = Z2 - M_hat2
    
    M1 = M_hat1 + np.dot(K1,mu1)
    M2 = M_hat2 + np.dot(K2,mu2)
    P1 = np.dot(H-np.dot(K1,H),P_hat1)  #Actual formula is (I-K*H)*P_hat, H is just used to as it is identity
    P2 = np.dot(H-np.dot(K2,H),P_hat2)
    
    if(M1[1,0]>=6.28):
        M1[11,0] = M1[1,0]-6.28
    elif(M1[1,0]<0):
        M1[1,0] = M1[1,0]+6.28
        
    if(M2[1,0]>=6.28):
        M2[1,0] = M2[1,0]-6.28
    elif(M2[1,0]<0):
        M2[1,0] = M2[1,0]+6.28
    
    return d2_hat1,d2_hat2,M1,M2,P1,P2  
  
#############################Edit only this function######

def lidar_callback(msg):
  global tlidar_data, lidar_data, d, P_prev1, P_prev2, M_prev1, M_prev2, ite, V_prev,t_l_1
  #Make sense of the callback data
  #rostopic echo msg
  #0 to 359 aray index values with each value at a degree increment 
  #range should be in meters
  #The readings start from left and go counter clockwise
  t_l_sec = msg.header.stamp.secs
  t_l_nsec = msg.header.stamp.nsecs
  t_l = t_l_sec + t_l_nsec*10**(-9)
  dt = t_l-t_l_1
  RANGES=list(msg.ranges)
  
  d = 0.5
  
  lidar_data.append(RANGES)
  tlidar_data.append(t_l)
  
  if (t_l > tlidar_data[0]+8):
    lidar_mat = np.array(lidar_data)
    tlidar_mat = np.array(tlidar_data)
    
    scipy.io.savemat('lid_robot2_FORM3.mat', dict(t1lid=tlidar_mat, lid=lidar_mat))
    
  RANGES1=list(msg.ranges)
    
  i=0
  while(i<len(RANGES)):
    if(RANGES[i]<=msg.range_min or RANGES[i]>=1.3):
      RANGES[i]=10000
      RANGES1[i]=10000 
    i+=1
    
  min_value_1 = min(RANGES)
  min_index_1 = RANGES.index(min_value_1)
  
  Z1 = [[min_value_1],[min_index_1*np.pi/180]] 
  
  Z1_data.append(Z1) 
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
  
  Z1 = np.array(Z1)
  Z2 = np.array(Z2) 
  
  if(ite == 0):
    M_prev1 = Z1
    M_prev2 = Z2
  
  ite = ite+1
  
  d1,d2,M_prev1,M_prev2,P_prev1,P_prev2 = LidarMin2(P_prev1,P_prev2,M_prev1,M_prev2,V_prev,Z1,Z2,robot_theta*np.pi/180,dt)
  M1_data.append(list(M_prev1))
  M2_data.append(list(M_prev2))
  d1_data.append(d1)
  d2_data.append(d2)
  
  if (t_l > tlidar_data[0]+8):
    lidarM1_mat = np.array(M1_data)
    lidarM2_mat = np.array(M2_data)
    lidarZ1_mat = np.array(Z1_data)
    lidarZ2_mat = np.array(Z2_data)
    lidard1_mat = np.array(d1_data)
    lidard2_mat = np.array(d2_data)
    
    scipy.io.savemat('Kallid_robot2_FORM3.mat', dict(lidM1=lidarM1_mat, lidM2=lidarM2_mat, lidZ1=lidarZ1_mat, lidZ2=lidarZ2_mat, lidd1=lidard1_mat, lidd2=lidard2_mat ))
  
  print(" Measurement1 " , Z1," Measurement2 " , Z2)
  print(" Prediction1 " , M_prev1," Prediction2 " , M_prev2)
  
  controller()
  t_l_1 = t_l
          
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
    
    scipy.io.savemat('odom_robot2_FORM3.mat', dict(t1=todom_mat, x1=x_mat, y1=y_mat, th1=theta_mat))
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


