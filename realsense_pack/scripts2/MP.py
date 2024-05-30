#!/usr/bin/env python

import rospy
import math
import heapq
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import tf

class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_planner', anonymous=True)

        # Parameters
        self.rate = rospy.Rate(10)
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None

        # Subscribers
        rospy.Subscriber('/occupancy_grid', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odom', Odometry, self.pose_callback)
        
        # Publishers
        self.path_pub = rospy.Publisher('/astar_path', Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.listener = tf.TransformListener()

        self.current_pose = None
        
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = 2.0  # Set your goal x position
        self.goal_pose.pose.position.y = 2.0  # Set your goal y position
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation = quaternion_from_euler(0, 0, 0)

    def map_callback(self, data):
        self.map_data = data.data
        self.resolution = data.info.resolution
        self.origin = data.info.origin
        self.width = data.info.width
        self.height = data.info.height

    def pose_callback(self, data):
        self.current_pose = data.pose.pose

    def a_star(self, start, goal):
        def heuristic(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        def get_neighbors(node):
            neighbors = [
                (node[0] + 1, node[1]), (node[0] - 1, node[1]),
                (node[0], node[1] + 1), (node[0], node[1] - 1),
                (node[0] + 1, node[1] + 1), (node[0] - 1, node[1] - 1),
                (node[0] + 1, node[1] - 1), (node[0] - 1, node[1] + 1)
            ]
            return [neighbor for neighbor in neighbors if self.is_within_bounds(neighbor) and self.is_free(neighbor)]

        def reconstruct_path(came_from, current):
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(current)
            path.reverse()
            print(path)
            return path

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return reconstruct_path(came_from, current)

            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def is_within_bounds(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, node):
        x, y = node
        index = y * self.width + x
        return self.map_data[index] == 0

    def world_to_map(self, pose):
        x = int((pose.position.x - self.origin.position.x) / self.resolution)
        y = int((pose.position.y - self.origin.position.y) / self.resolution)
        return x, y

    def map_to_world(self, node):
        x = node[0] * self.resolution + self.origin.position.x
        y = node[1] * self.resolution + self.origin.position.y
        return x, y

    def run(self):
        while not rospy.is_shutdown():
            if self.map_data is not None and self.current_pose is not None:
                start = self.world_to_map(self.current_pose)
                goal = self.world_to_map(self.goal_pose.pose)  # Set your goal pose here

                path = self.a_star(start, goal)
                if path:
                    path_msg = Path()
                    path_msg.header.frame_id = "map"
                    for node in path:
                        pose = PoseStamped()
                        pose.pose.position.x, pose.pose.position.y = self.map_to_world(node)
                        path_msg.poses.append(pose)
                    self.path_pub.publish(path_msg)

                    # Move the TurtleBot along the path
                    self.move_along_path(path)

            self.rate.sleep()

    def move_along_path(self, path):
        for node in path:
            target_x, target_y = self.map_to_world(node)
            while not rospy.is_shutdown():
                trans = None
                try:
                    now = rospy.Time(0)
                    self.listener.waitForTransform("/base_link", "/occupancy_grid", now, rospy.Duration(1.0))
                    (trans, rot) = self.listener.lookupTransform("/base_link", "/occupancy_grid", now)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                angular = math.atan2(target_y - trans[1], target_x - trans[0]) - tf.transformations.euler_from_quaternion(rot)[2]
                if angular > math.pi:
                    angular -= 2 * math.pi
                elif angular < -math.pi:
                    angular += 2 * math.pi

                distance = math.sqrt((target_x - trans[0]) ** 2 + (target_y - trans[1]) ** 2)
                if distance < 0.1:
                    break

                cmd_vel = Twist()
                cmd_vel.linear.x = min(0.2, distance)
                cmd_vel.angular.z = min(0.5, max(-0.5, angular))
                self.cmd_vel_pub.publish(cmd_vel)
                self.rate.sleep()

if __name__ == '__main__':
    try:
        astar_planner = AStarPlanner()
        astar_planner.run()
    except rospy.ROSInterruptException:
        pass

