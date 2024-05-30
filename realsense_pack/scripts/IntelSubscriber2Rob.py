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

global s, id_list, Coordinates_list,z_anglist,ite,initial_time,move1,move2,flag1,flag2,flag3,poseid,poseidin,goalx1,goaly1,goalx2,goaly2
s = 0.5
id_list = None
Coordinates_list = None
z_anglist = None
poseid = None
poseidin = None

id_mat = []
Coordinates_mat = []
z_ang_mat = []
rgb_mat = []
time_mat = []
ctime_mat1 = []
c_mat1 = []
dist_mat1 = []
ctime_mat2 = []
c_mat2 = []
dist_mat2 = []
ite = 0
initial_time=0
flag1 = 0
flag2 = 0
flag3 = 0
goalx1 = 0.6
goaly1 = 0.6
goalx2 = 0
goaly2 = 1.0

# Commanded velocity 
move1 = Twist() # defining the variable to hold values
move1.linear.x = 0
move1.linear.y = 0
move1.linear.z = 0
move1.angular.x = 0
move1.angular.y = 0
move1.angular.z = 0

move2 = Twist() # defining the variable to hold values
move2.linear.x = 0
move2.linear.y = 0
move2.linear.z = 0
move2.angular.x = 0
move2.angular.y = 0
move2.angular.z = 0

class IntelSubscriber:
    global ite
    def __init__(self):
       rospy.init_node('realsense_subscriber_rgb',anonymous=True)
       
       self.bridge = CvBridge()
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
    
    def IntelSubscriberRGB(self,msg):
       global ite,flag1,flag2,flag3,initial_time,flag,initial_time,move1,move2,ctime_mat1,c_mat1,dist_mat1,ctime_mat2,c_mat2,dist_mat2,RelPosition,poseidin, goalx1, goaly1,goalx2, goaly2,xin1,xin2,yin1,yin2,zangin1,zangin2
       if ite == 0:
          initial_time = time.time()
          time_mat.append(initial_time)   
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
          rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
          cv2.waitKey(1)
       except Exception as e:
          print(e)
          
       camera_msg = rospy.wait_for_message('/realsense/camera/rgb/camera_info', CameraInfo)
       depth_msg = rospy.wait_for_message('/realsense/camera/depth/image_raw', Image)
       depth_image = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='passthrough')
       
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
             if ids[i,0] == 1:
                rvec, tvec,z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                p1 = center
                depth_value = depth_image[center[1], center[0]]
                
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
                   x1_ = depth_point[0]
                   y1_ = depth_point[1]
                   z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                   org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
             
             elif ids[i,0] == 2:
                rvec, tvec, z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                depth_value = depth_image[center[1], center[0]]
                p2 = center
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
                   x2_ = depth_point[0]
                   y2_ = depth_point[1]
                   z2 = depth_point[2]
                   org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
          x1_, y1_, x2_,y2_ = round(x1_,2),round(y1_,2), round(x2_,2), round(y2_,2)
          calib_coords = [x1_, y1_, x2_,y2_]
          theta = self.angle(y1_,y2_,x1_,x2_)

          if (1 in ids) and (2 in ids):
              x1,y1 = self.coordinates(calib_coords,x1_,y1_,theta)
              x2,y2 = self.coordinates(calib_coords,x2_,y2_,theta)
              marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
              marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
              cv2.line(rgb_image, p1, p2,(255,0,0), 1)
              #cv2.putText(color_image, marker_text_a, org_a, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
              #cv2.putText(color_image, marker_text_b, org_b, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
          else:
              print("Reference tags are not in view")
       id_list = []
       Coordinates_list = []
       z_anglist = []
       poseid = []                 
       if ids is not None:
          for i in range(len(ids)):
             rvec, tvec, z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix,None)
             center = np.mean(corners[i][0], axis=0).astype(int)
             depth_value = depth_image[center[1], center[0]]
             
             if depth_value > 0:
                depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)

                marker_text = "Marker ID: {} | Coordinates: {:.2f}, {:.2f}, {:.2f}".format(ids[i][0], depth_point[0], depth_point[1], depth_point[2])
                   
                marker_x_ = depth_point[0]
                marker_y_ = depth_point[1]
                marker_z = depth_point[2]
                  
                #using the calibrated values
                marker_x,marker_y = self.coordinates(calib_coords,marker_x_,marker_y_,theta)
                id_list.append(ids[i,0])
                Coordinates_list.append([marker_x,marker_y])
                
                z_ang = z_ang-np.pi/2 
                  
                if z_ang<0:
                   z_ang = z_ang+2*np.pi
                z_anglist.append(z_ang*180/np.pi)
                
                poseid.append([ids[i,0],marker_x,marker_y,z_ang])
                
                # Robot ids
                
                if ids[i,0] == 5 and flag1 == 0:
                   print("yes")
                   xin1 = marker_x
                   yin1 = marker_y
                   zangin1 = z_ang
                   flag1 = 1
                elif ids[i,0] == 5 and flag1 == 1:
                   x11 = marker_x
                   y11 = marker_y
                   zangle1 = z_ang
                   
                if ids[i,0] == 4 and flag2 == 0:
                   print("yes")
                   xin2 = marker_x
                   yin2 = marker_y
                   zangin2 = z_ang
                   flag2 = 1
                elif ids[i,0] == 4 and flag2 == 1:
                   x22 = marker_x
                   y22 = marker_y
                   zangle2 = z_ang
                
                marker_text = "ID: {} ({:.2f}, {:.2f})".format(ids[i][0], marker_x, marker_y)
                # Convert coordinates to integers before passing to putText
                org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                cv2.putText(rgb_image, marker_text, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.aruco.drawDetectedMarkers(rgb_image, corners)
       print("id list",id_list,"Coordinates list",Coordinates_list,"z_anglist",z_anglist)
       if flag3 == 0:
          poseidin = poseid
          flag3 == 1
          
       cv2.imshow("ArUco Marker Detection", rgb_image)
       # Exit the program when the 'Esc' key is pressed
       #if cv2.waitKey(1) & 0xFF == 27:
          #break
       #RelPosition = self.RelPos(poseid)
       #print(RelPosition)
          
       id_mat.append(id_list)
       Coordinates_mat.append(Coordinates_list)
       z_ang_mat.append(z_anglist)
       #rgb_mat.append(rgb_image)
       
       current_time = time.time()
       print("ite ",ite," initial time ",initial_time," current time ",current_time)
       
       ite = ite+1
       
       time_mat.append(current_time)
       
       self.controller(x11,y11,zangle1,xin1,yin1,zangin1,ctime_mat1,c_mat1,dist_mat1,move1,goalx1,goaly1)
       self.controller(x22,y22,zangle2,xin2,yin2,zangin2,ctime_mat2,c_mat2,dist_mat2,move2,goalx2,goaly2)
       
       if (current_time > initial_time + 8 ):
          id_mat1 = np.array(id_mat)
          Coordinates_mat1 = np.array(Coordinates_mat)
          z_ang_mat1 = np.array(z_ang_mat)
          #rgb_mat1 = np.array(rgb_mat)
          time_mat1 = np.array(time_mat)
          c_mat11 = np.array(c_mat1)
          ctime_mat11 = np.array(ctime_mat1)
          dist_mat11 = np.array(dist_mat1)
          c_mat22 = np.array(c_mat2)
          ctime_mat22 = np.array(ctime_mat2)
          dist_mat22 = np.array(dist_mat2)
          scipy.io.savemat('Aruco.mat', dict(idmat=id_mat1, Coodmat=Coordinates_mat1, zmat=z_ang_mat1, cmat1=c_mat11, ctimemat1 = ctime_mat11, timemat = time_mat1,distmat1 = dist_mat11, cmat2=c_mat22, ctimemat2 = ctime_mat22, distmat2 = dist_mat22))

    def coordinates(self, calib_coords,x_,y_,theta):
       x1,y1,x2,y2 = calib_coords
       tan = np.tan(theta)
       cos = np.cos(theta)
       sec = 1/(np.cos(theta))
       cosec = 1/(np.sin(theta))
       y_tmp = (y_ - y1 - (x_- x1)*tan)*sec
       x_tmp = (x_ - x1)*sec + y_tmp*tan
       s_ = np.sqrt((y2-y1)**2 + (x2-x1)**2)
       dist_fac = s_/s
       x = x_tmp/dist_fac
       y = y_tmp/dist_fac
       return x,y
    
    def angle(self, y1_,y2_,x1_,x2_):
       tmp = float(y2_-y1_)/float(x2_ - x1_)
       invtan = np.arctan(tmp)
       return invtan
    
    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
       marker_points = np.array([[-marker_size / 2, marker_size / 2, 0], [marker_size / 2, marker_size / 2, 0], [marker_size / 2, -marker_size / 2, 0], [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
       trash = []
       rvecs = []
       tvecs = []
       z_angle = 0
       for c in corners:
          nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
          RR,_ = cv2.Rodrigues(R)
          sy = np.sqrt(RR[0,0]*RR[0,0]+RR[1,0]*RR[1,0])
          if sy>1e-6:
              z_angle = z_angle+np.arctan2(RR[1,0],RR[0,0])
          else:
              z_angle = z_angle
          
          rvecs.append(R)
          tvecs.append(t)
          trash.append(nada)
       #z_angle = z_angle/4
       return rvecs, tvecs, z_angle, trash
    
    def controller(self,x,y,zang,xin,yin,zangin,ctime_mat,c_mat,dist_mat,move,goalx,goaly):
       alpha = 0.1
       k = 0.1
       #goalx = 0.6#xin + 0.6*np.cos(zangin+np.pi/4)
       #goaly = 0.6#yin + 0.6*np.sin(zangin+np.pi/4)
       dist = np.sqrt((goalx-x)**2 +(goaly-y)**2)
       desired_orientation = np.arctan2((goaly - y),(goalx - x))
       orientation_error = desired_orientation - zang
       orientation_error%=(2*np.pi)
       if orientation_error>np.pi:
          orientation_error-= 2*np.pi
          
       if np.absolute(dist) > 0.01:
          move.linear.x = alpha*dist
          move.angular.z = -k*orientation_error
       else:
          move.linear.x = 0
          move.angular.z = 0
       ctime = time.time()
       dist_mat.append(dist)
       c_mat.append([move.linear.x,move.angular.z])
       ctime_mat.append(ctime)
       
    def RelPos(self, poseid):
       RelPosition = None
       pose_sort = poseid[poseid[:, 0].argsort()]
       for i in range(len(pose_sort)):
          for j in range(len(pose_sort)):
             RelPosition[i*len(pose_sort)+j,0] = pose_sort[i,1]-pose_sort[j,1]
             RelPosition[i*len(pose_sort)+j,1] = pose_sort[i,2]-pose_sort[j,2]
       return RelPosition  
       

if __name__ == '__main__':
    try:
       velocity_pub1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
       velocity_pub2 = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=10)
       rs_subscriber = IntelSubscriber()
       rate = rospy.Rate(5)
       while not rospy.is_shutdown():
          velocity_pub1.publish(move1)
          velocity_pub2.publish(move2)
          rate.sleep()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
