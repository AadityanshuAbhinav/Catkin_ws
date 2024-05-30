#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class LidarToOccupancyGrid:
    def __init__(self):
        rospy.init_node('lidar_to_occupancy_grid', anonymous=True)

        # Parameters
        self.resolution = 0.1  # Resolution of the grid in meters/cell
        self.grid_width = 200  # Width of the grid in cells
        self.grid_height = 200  # Height of the grid in cells
        self.origin = Pose()
        self.origin.position.x = -self.grid_width * self.resolution / 2
        self.origin.position.y = -self.grid_height * self.resolution / 2
        self.origin.position.z = 0
        self.origin.orientation = quaternion_from_euler(0, 0, 0)

        # Occupancy grid
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = "map"
        self.occupancy_grid.info.resolution = self.resolution
        self.occupancy_grid.info.width = self.grid_width
        self.occupancy_grid.info.height = self.grid_height
        self.occupancy_grid.info.origin = self.origin
        self.occupancy_grid.data = [-1] * (self.grid_width * self.grid_height)

        # Subscribers and Publishers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)

        self.rate = rospy.Rate(1)  # Publish rate in Hz

    def scan_callback(self, scan):
        self.update_occupancy_grid(scan)

    def update_occupancy_grid(self, scan):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        for i, distance in enumerate(scan.ranges):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = angles[i]
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            grid_x = int((x - self.origin.position.x) / self.resolution)
            grid_y = int((y - self.origin.position.y) / self.resolution)

            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                index = grid_y * self.grid_width + grid_x
                self.occupancy_grid.data[index] = 100  # Mark as occupied

        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.occupancy_grid)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        lidar_to_occupancy_grid = LidarToOccupancyGrid()
        lidar_to_occupancy_grid.run()
    except rospy.ROSInterruptException:
        pass

