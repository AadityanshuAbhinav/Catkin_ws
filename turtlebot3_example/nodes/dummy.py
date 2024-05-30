#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from sensor_msgs.msg import LaserScan

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class PathPlanner():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        self.move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.MIN_LIMIT = 0.3

        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            #self.shutdown()
        goal_z = np.deg2rad(goal_z)
        if not self.obstacle_detector():
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_footprint'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                try:
                    self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                    self.base_frame = 'base_link'
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                    rospy.signal_shutdown("tf Exception")
    
            (position, rotation) = self.get_odom()
            last_rotation = 0
            linear_speed = 1
            angular_speed = 1
            
            goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
            distance = goal_distance
    
            while distance > 0.05:
                (position, rotation) = self.get_odom()
                x_start = position.x
                y_start = position.y
                path_angle = atan2(goal_y - y_start, goal_x- x_start)
    
                if path_angle < -pi/4 or path_angle > pi/4:
                    if goal_y < 0 and y_start < goal_y:
                        path_angle = -2*pi + path_angle
                    elif goal_y >= 0 and y_start > goal_y:
                        path_angle = 2*pi + path_angle
                if last_rotation > pi-0.1 and rotation <= 0:
                    rotation = 2*pi + rotation
                elif last_rotation < -pi+0.1 and rotation > 0:
                    rotation = -2*pi + rotation
                self.controller(0,angular_speed*path_angle-rotation) 
    
                distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
                self.controller(0.225,0)
                
                #print('Angular Velocity: ', self.move_cmd.angular.z)
    
                if self.move_cmd.angular.z > 0:
                    self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
                else:
                    self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)
    
                last_rotation = rotation
                self.cmd_vel.publish(self.move_cmd)
                r.sleep()
                if self.obstacle_detector():
                    rospy.loginfo("Obstacle detected")
                    self.stop_robot()
                    self.deal_obstacle()
                    #self.cmd_vel.publish(Twist())
                    return
    
            (position, rotation) = self.get_odom()
            
    
            while abs(rotation - goal_z) > 0.05:
                (position, rotation) = self.get_odom()
                if goal_z >= 0:
                    if rotation <= goal_z and rotation >= goal_z - pi:
                    	 self.controller(0,0.5)
             
                    else:
                        self.controller(0,-0.5)
                else:
                    if rotation <= goal_z + pi and rotation > goal_z:
                        self.controller(0,-0.5)
                    else:
                        self.controller(0,0.5)
                #self.cmd_vel.publish(self.move_cmd)
                #print('Linear Velocity: ', self.move_cmd.linear.x)
                #print('Angular Velocity: ', self.move_cmd.angular.z)
                r.sleep()
        else:
            rospy.loginfo("Obstacle detected")
            self.stop_robot()
            self.deal_obstacle()
            self.cmd_vel.publish(self.move_cmd)
            return

        rospy.loginfo("Reached the goal point... Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        print('get_odom')
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def obstacle_detector(self):
        #print('obstacle_detector')
        scan = rospy.wait_for_message('scan', LaserScan)
        lidar_distances = scan.ranges
        ranges = lidar_distances
        

        angle_min = 0
        angle_max = 359
        angle_increment = 1
        left = ranges[0:46]
        right = ranges[314:]
        left_min = min(left)
        right_min = min(right)
        if (left_min < self.MIN_LIMIT) or (right_min < self.MIN_LIMIT):
            return True
            if left_min < self.MIN_LIMIT:
               print('Obstacle on left')
            if right_min < self.MIN_LIMIT:
               print('Obstacle on right')
               

        # Convert angles to indices in ranges array
        #index_min = angle_min #int(((-45 * 3.14159 / 180) - angle_min) / angle_increment)
        #index_max = angle_max #int(((45 * 3.14159 / 180) - angle_min) / angle_increment)

        # Check distances in the specified range
        #for i in range(index_min, index_max + 1):
            #if ranges[i] < SAFE_STOP_DISTANCE:
                #print("Obstacle detected at {:.2f} degrees. Distance: {:.2f} meters".format(
                    #angle_min + i * angle_increment, ranges[i]))
                #return True
        return False

    def deal_obstacle(self):
        #print('deal_obstacle')
        scan = rospy.wait_for_message('scan', LaserScan)
        lidar_distances = scan.ranges
        ranges = scan.ranges
        left_view = list(ranges[0:91])
        left_most=list(ranges[88:93])
        right_view =list(ranges[270:])
        right_most =list(ranges[268:273])

        i=0
        while(i<len(left_view)):
            if(left_view[i]<=scan.range_min):
                left_view[i]=10000
            i+=1
        i=0
        while(i<len(right_view)):
            if(right_view[i]<=scan.range_min):
                right_view[i]=10000
            i+=1
            
        while(i<len(left_most)):
            if(left_most[i]<=scan.range_min):
                left_most[i]=10000
            i+=1
        i=0
        while(i<len(right_most)):
            if(right_most[i]<=scan.range_min):
                right_most[i]=10000
            i+=1
            
        min_range_left=min(left_view)
        min_left_ind = left_view.index(min_range_left)
        turn_left_ang = 270 - min_left_ind
        turn_left_rad = (turn_left_ang/180)*3.14
        
        
        
        min_range_right=min(right_view)
        min_right_ind = right_view.index(min_range_right)
        turn_right_ang = 90 - min_right_ind
        turn_right_rad = (turn_right_ang/180)*3.14
        
        left_most = min(left_most)
        right_most = min(right_most)
        flag = 0
        print('Rounding the obstacle')
        while self.obstacle_detector() is True:
            
            if min_range_left < self.MIN_LIMIT and min_range_right > self.MIN_LIMIT:
                flag =1
                print("Turning right")
                self.controller(0.05,-0.225)
            elif min_range_left > self.MIN_LIMIT and min_range_right < self.MIN_LIMIT:
                print("Turning Left")
                flag =2
                self.controller(0.05,0.225)
            elif (min_range_left == min_range_right) and  min_range_right < self.MIN_LIMIT:
                print('Turning right')
                flag = 1
                self.controller(0.05,-0.225)
            else:
                pass
            self.controller(0.1,0)
            self.cmd_vel.publish(self.move_cmd)
            self.deal_obstacle()
            
        print('Obstacle Overcame')
        return
        
        
            

        #self.Controller()

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
    def controller(self,vel,ang):
        self.move_cmd.linear.x = vel
        self.move_cmd.angular.z = ang
        
        self.cmd_vel.publish(self.move_cmd)
        
        print('Linear Velocity: ', self.move_cmd.linear.x)
        print('Angular Velocity: ', self.move_cmd.angular.z)
        
      
    def stop_robot(self):
        
        print('stop_robot')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel.publish(stop_msg)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            print(msg)
            PathPlanner()

    except Exception as e:
        rospy.logerr('error: {}'.format(e))
        import traceback
        traceback.print_exc()