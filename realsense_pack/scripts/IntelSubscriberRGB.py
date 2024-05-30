#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import scipy.io


rgb_data = []
depth_data = []
ite1_data = []
ite2_data = []
ite1 = 0
ite2 = 0

class IntelSubscriber:
    def __init__(self):
       rospy.init_node('realsense_subscriber_rgb',anonymous=True)
       self.bridge = CvBridge()
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
       #self.depth_sub = rospy.Subscriber('/realsense/camera/depth/image_raw', Image, self.IntelSubscriberDepth)
    
    def IntelSubscriberRGB(self,msg):
       global RGB_image,rgb_data,ite1_data,ite1
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
          cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
          print("RGB",cv_image)
          cv2.imshow("RGB",cv_image)
          cv2.waitKey(1)
       except Exception as e:
          print(e)
          #rgb_data.append(RGB_image)
          #rgb_mat = np.array(rgb_data)
          #ite1_data.append(ite1)
          #ite1_mat = np.array(ite1_data)
          #ite1 = ite1 +1
          #scipy.io.savemat('CamRGB.mat', dict(rgb=rgb_mat,ite1 = ite1_mat))
    
    def IntelSubscriberDepth(self,msg):
       global Depth_image,depth_data,ite2_data,ite2
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving depth data")
          cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
          print("Depth",cv_image)
          cv2.waitKey(1)
       except Exception as e:
          print(e)
    #depth_data.append(Depth_image)
    #depth_mat = np.array(depth_data)
    #ite2_data.append(ite2)
    #ite2_mat = np.array(ite2_data)
    #ite2 = ite2 + 1
    #scipy.io.savemat('CamDepth.mat', dict(depth=depth_mat))   

if __name__ == '__main__':
    try:
       rs_subscriber = IntelSubscriber()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
