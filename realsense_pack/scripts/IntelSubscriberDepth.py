#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import scipy.io
import cv2.aruco as aruco


rgb_data = []
depth_data = []
ite1_data = []
ite2_data = []
ite1 = 0
ite2 = 0
rgb_Image = []
depth_Image = []
intrinsics = None

class IntelSubscriber:
    def __init__(self):
       global rgb_image,depth_image 
       rospy.init_node('realsense_subscriber_depth',anonymous=True)
       self.bridge = CvBridge()
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
       self.depth_sub = rospy.Subscriber('/realsense/camera/depth/image_raw', Image, self.IntelSubscriberDepth)
       #print(rgb_image)
    
    def IntelSubscriberRGB(self,msg):
       global rgb_image,rgb_data,ite1_data,ite1
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
          rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
          #print("RGB",rgb_image)
          #cv2.imshow("RGB",rgb_image)
          #cv2.waitKey(1)
       except Exception as e:
          print(e)
       
       depth_msg = rospy.wait_for_message('/realsense/camera/depth/image_raw', Image)
       depth_image = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='passthrough')   
       
       aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
       aruco_params = cv2.aruco.DetectorParameters_create()
       
       corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, aruco_dict, parameters=aruco_params)
       
       if ids is not None:
          for i in range(len(ids)):
             if ids[i] == 1:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                p1 = center
                depth_value = depth_image[center[1], center[0]]
                p1 = center
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                   x1_ = depth_point[0]
                   y1_ = depth_point[1]
                   z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                   org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

             elif ids[i] == 2:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                depth_value = depth_frame.get_distance(center[0], center[1])
                p2 = center
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)
                   
                   x2_ = depth_point[0]
                   y2_ = depth_point[1]
                   z2 = depth_point[2]
                   org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10) 

             x1_, y1_, x2_,y2_ = round(x1_,2),round(y1_,2), round(x2_,2), round(y2_,2)
             calib_coords = [x1_, y1_, x2_,y2_]
             theta = angle(y1_,y2_,x1_,x2_)

             if (1 in ids) and (2 in ids):
                 x1,y1 = coordinates(calib_coords,x1_,y1_,theta)
                 x2,y2 = coordinates(calib_coords,x2_,y2_,theta)
                 marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
                 marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
                 cv2.line(color_image, p1, p2,(255,0,0), 1)
                 #cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                 #cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
             else:
                 break

       if ids is not None:
          for i in range(len(ids)):
             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, np.eye(3), None)
             center = np.mean(corners[i][0], axis=0).astype(int)
             depth_value = depth_frame.get_distance(center[0], center[1])
             
             if depth_value > 0:
                depth_point = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().intrinsics, [center[0], center[1]], depth_value)

                marker_text = "Marker ID: {} | Coordinates: {:.2f}, {:.2f}, {:.2f}".format(ids[i][0], depth_point[0], depth_point[1], depth_point[2])
                   
                marker_x_ = depth_point[0]
                marker_y_ = depth_point[1]
                marker_z = depth_point[2]
                  
                #using the calibrated values
                marker_x,marker_y = coordinates(calib_coords,marker_x_,marker_y_,theta)
                print(ids[i]," ", marker_x, " ",marker_y)
                marker_text = "ID: {} ({:.2f}, {:.2f})".format(ids[i][0], marker_x, marker_y)
                # Convert coordinates to integers before passing to putText
                org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                cv2.putText(color_image, marker_text, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.aruco.drawDetectedMarkers(color_image, corners)

       # cv2.imshow("ArUco Marker Detection", color_image)         
          #rgb_data.append(RGB_image)
          #rgb_mat = np.array(rgb_data)
          #ite1_data.append(ite1)
          #ite1_mat = np.array(ite1_data)
          #ite1 = ite1 +1
          #scipy.io.savemat('CamRGB.mat', dict(rgb=rgb_mat,ite1 = ite1_mat))
    
    def IntelSubscriberDepth(self,msg):
       global depth_image,depth_data,ite2_data,ite2
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving depth data")
          depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
          #print("Depth",depth_image)
          #cv2.imshow("Depth",depth_image)
          cv2.waitKey(1)
       except Exception as e:
          print(e)
	    #depth_data.append(Depth_image)
	    #depth_mat = np.array(depth_data)
	    #ite2_data.append(ite2)
	    #ite2_mat = np.array(ite2_data)
	    #ite2 = ite2 + 1
	    #scipy.io.savemat('CamDepth.mat', dict(depth=depth_mat))

def camera_callback(msg):
    global intrinsics
    intrinsics = msg
    

if __name__ == '__main__':
    try:
       rs_subscriber = IntelSubscriber()
       print(rgb_Image)
       #rs_subscriber.ArucoDetector(rgb_Image,depth_Image)
       #print(rs_subscriber.rgb_image)
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
