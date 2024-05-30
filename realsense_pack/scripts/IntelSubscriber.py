#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import scipy.io
import cv2.aruco as aruco
import time
import scipy.io
from geometry_msgs.msg import Twist

global ite,initial_time

ite = 0
initial_time=0

class IntelSubscriber:
    global ite,initial_time
    def __init__(self):
       rospy.init_node('realsense_subscriber_rgb',anonymous=True)
       
       self.bridge = CvBridge()
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
    
    def IntelSubscriberRGB(self,msg):
       global ite,flaglist,initial_time,move1,move2,move3,ctime_mat1,c_mat1,dist_mat1,ctime_mat2,c_mat2,dist_mat2,ctime_mat3,c_mat3,dist_mat3,RelPosition,poseidin, goallist,Form,FormErrMat1,DistErrMat1,FormErrMat2,DistErrMat2,FormErrMat3,DistErrMat3,PI_error1,PI_error2,PI_error3,v
       if ite == 0:
          initial_time = time.time()
          #time_mat.append(initial_time) 
            
       try:
          rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
       except Exception as e:
          print(e)
          
       camera_msg = rospy.wait_for_message('/realsense/camera/rgb/camera_info', CameraInfo)
       
       camera_matrix = np.array(camera_msg.K).reshape((3,3))
       dist_coeff = np.array(camera_msg.D)
       intrinsics = rs.intrinsics()
       intrinsics.width = 640
       intrinsics.height = 480
       intrinsics.fx = camera_matrix[0,0]
       intrinsics.fy = camera_matrix[1,1]
       intrinsics.ppx = camera_matrix[0,2]
       intrinsics.ppy = camera_matrix[1,2]
       
       aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
       aruco_params = cv2.aruco.DetectorParameters()
       aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
       
       corners, ids, _ = aruco_detector.detectMarkers(rgb_image)
       
       if ids is not None:
        for i in range(len(ids)):
            if (ids[i] == 1):
                ref1 = corners[i]
                pre1 = 1
                
        CamPose1, Camrot1, tvecref1 = self.my_estimateCameraPose(pre1, ref1, 0.1,camera_matrix, None)
        print(CamPose1)
                
        for i in range(len(ids)):       
            rvec, tvec, RRmat, TTmat, Eulmat, _ = self.my_estimatePoseSingleMarkers2(corners[i], 0.1, camera_matrix, None,CamPose1,Camrot1)
            print(ids[i], " " , TTmat)
    
    
    def my_estimatePoseSingleMarkers2(self,corners, marker_size, mtx, distortion,CamPose,CamRot):
       marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],[marker_size / 2, marker_size / 2, 0],[marker_size / 2, -marker_size / 2, 0],[-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
       trash = []
       rvecs = []
       tvecs = []
       RRmat = []
       TTmat = []
       Eulmat = []
       LL = np.array([[0,0,0,1]])
       H_CW = np.concatenate((np.concatenate((CamRot, CamPose), axis=1),LL),axis=0)
       
       nada, R, t = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
       RR,_ = cv2.Rodrigues(R)
       H_TC = np.concatenate((np.concatenate((RR, t), axis=1),LL),axis=0)
       H_TW = np.matmul(H_TC,H_CW)
       
       T_TW = H_TW[0:3,3]
       R_TW = H_TW[0:3,0:3]
       
       ypr = cv2.RQDecomp3x3(R_TW)
       
       rvecs.append(R)
       tvecs.append(t)
       TTmat.append(T_TW)
       RRmat.append(R_TW)
       Eulmat.append(ypr)
       trash.append(nada)
       
       return rvecs, tvecs, RRmat, TTmat, Eulmat, trash
    
    def my_estimateCameraPose(self,pre, ref,marker_size, mtx, distortion):
       marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],[marker_size / 2, marker_size / 2, 0],[marker_size / 2, -marker_size / 2, 0],[-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
       trash = []
       rvecs = []
       tvecs = []
       RRmat = []
       Eulmat = []
       
       if pre == 1:
          nadaref, rvecref, tvecref = cv2.solvePnP(marker_points, ref, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
          RRref,_ =  cv2.Rodrigues(rvecref)
       else:
          tvecref = np.zeros((3,1),dtype=float)
          RRref = np.eye(3, dtype = float)
          
       cameraPose = -np.matrix(RRref).T * np.matrix(tvecref)
       cameraRot = np.matrix(RRref).T
       
       return cameraPose, cameraRot, tvecref
          
          
if __name__ == '__main__':
    try:
       #velocity_pub1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
       #velocity_pub2 = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
       #velocity_pub3 = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=10)
       rs_subscriber = IntelSubscriber()
       #rate = rospy.Rate(10)
       #while not rospy.is_shutdown():
          #velocity_pub1.publish(move1)
          #velocity_pub2.publish(move2)
          #velocity_pub3.publish(move3)
          #rate.sleep()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
