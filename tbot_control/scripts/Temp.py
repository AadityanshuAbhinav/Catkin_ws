#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy as np
import scipy.io

posehist = np.zeros([3,3,3])

pose1 = np.array([[1,3,3],[3,3,4],[1,4,5]])
pose2 = np.array([[3,4,3],[2,3,4],[1,4,5]])
pose3 = np.array([[2,2,3],[2,3,4],[1,4,5]])
posef = np.zeros([3,3])
posehist[:,:,0] = pose1
posehist[:,:,1] = pose2
posehist[:,:,2] = pose3

def controller():
   global posehist,pose1,pose2,pose3,posef
   posef[:,2]=pose1[:,2]
   posef[:,0:2]=np.mean(posehist[:,0:2,:],axis=2)
   print(posef)
  
  
##########################################################

rospy.init_node('Circle')  # Defines a node with name of Follower

controller()


