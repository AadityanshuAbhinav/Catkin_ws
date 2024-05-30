#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
from heapq import heappush, heappop
import numpy as np
import math

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_turtlebot_planner', anonymous=True)

        self.map_width = 100
        self.map_height = 100
        self.resolution = 0.1
        self.origin = (-5, -5)

        self.grid = np.ones((self.map_height, self.map_width)) * -1
        self.start = None
        self.goal = None
        self.pose = None
        self.yaw = None

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/goal', PoseStamped, self.goal_callback)

        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def scan_callback(self, msg):
        self.grid.fill(-1)
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = int((self.pose[0] + r * math.cos(angle + self.yaw) - self.origin[0]) / self.resolution)
                y = int((self.pose[1] + r * math.sin(angle + self.yaw) - self.origin[1]) / self.resolution)
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    self.grid[y][x] = 0
            angle += msg.angle_increment

        self.publish_map()

    def odom_callback(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
        if self.start is None:
            self.start = self.world_to_map(self.pose)

    def goal_callback(self, msg):
        self.goal = self.world_to_map((msg.pose.position.x, msg.pose.position.y))
        self.plan_path()

    def world_to_map(self, position):
        mx = int((position[0] - self.origin[0]) / self.resolution)
        my = int((position[1] - self.origin[1]) / self.resolution)
        return (mx, my)

    def map_to_world(self, position):
        wx = position[0] * self.resolution + self.origin[0]
        wy = position[1] * self.resolution + self.origin[1]
        return (wx, wy)

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dx, dy in neighbors:
            x2, y2 = node[0] + dx, node[1] + dy
            if 0 <= x2 < self.grid.shape[1] and 0 <= y2 < self.grid.shape[0] and self.grid[y2][x2] == 0:
                result.append((x2, y2))
        return result

    def plan_path(self):
        if self.grid is None or self.start is None or self.goal is None:
            return

        start, goal = self.start, self.goal

        open_list = []
        heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            current = heappop(open_list)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                self.publish_path(path)
                self.follow_path(path)
                return

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    heappush(open_list, (f_score[neighbor], neighbor))

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.origin[0]
        map_msg.info.origin.position.y = self.origin[1]
        map_msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(map_msg)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for x, y in path:
            pose = PoseStamped()
            wx, wy = self.map_to_world((x, y))
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def follow_path(self, path):
        for x, y in path:
            wx, wy = self.map_to_world((x, y))
            goal_angle = math.atan2(wy - self.pose[1], wx - self.pose[0])
            distance = math.hypot(wx - self.pose[0], wy - self.pose[1])

            twist = Twist()
            while abs(goal_angle - self.yaw) > 0.1:
                twist.angular.z = 0.5 * (goal_angle - self.yaw)
                self.cmd_pub.publish(twist)
                rospy.sleep(0.1)

            while distance > 0.1:
                distance = math.hypot(wx - self.pose[0], wy - self.pose[1])
                twist.linear.x = 0.2 * distance
                self.cmd_pub.publish(twist)
                rospy.sleep(0.1)

        twist = Twist()
        self.cmd_pub.publish(twist)  # Stop the robot

if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


