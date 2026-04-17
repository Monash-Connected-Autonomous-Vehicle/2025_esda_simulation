#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import subprocess
import math

import rclpy

from rclpy.node import Node

# This node is the main node that will handle the overall track following around the entire map. It will subscribe to the laser scan data and implement the follow the gap algorithm to navigate around the track. It will also handle switching to a different navigation strategy when the robot is within a certain distance to the goal.
# The pipeline will be as follows:
# 1. Get the track direction --> Uses lane detection, obstacle detection and corridors from LiDAR data to determine the direction of the track and the location of the goal.
# 2. Follow the gap algorithm to navigate around the track until the robot is within a certain distance to the goal.
# 3. Converts to motion e.g. Angular Velocity, Linear Velocity to navigate towards the goal.
# 4. Publish /cmd_vel to move the robot.

class TrackFollower(Node):
    def __init__(self):
        super().__init__('track_follower')


        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_distance_threshold = 0.5  # Distance threshold to switch to goal navigation
        self.current_goal = None  # Placeholder for the current goal position

        # Subscribes to LiDAR laser scan data
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_listener_callback, 
            10
        )

        # Subscribes to lane detection data (if available) to assist in determining track direction
        self.lane_subscriber = self.create_subscription(
            MarkerArray, 
            '/lane_markers', 
            self.lane_listener_callback, 
            10
        )
    
    def lidar_listener_callback(self, msg):
        # Access LiDAR data from the LaserScan message
        ranges = msg.ranges

    def get_track_direction(self, scan_data):
        # Implement logic to determine the track direction based on LiDAR data
        pass



    def scan_callback(self, scan_data):
        # Process laser scan data and implement follow the gap logic here
        # This is where you would integrate the follow_the_gap algorithm


        pass

    def navigate_to_goal(self):
        # Implement navigation logic to move towards the current goal
        pass


def main(args=None):
    rclpy.init(args=args)
    track_follower = TrackFollower()
    rclpy.spin(track_follower)
    track_follower.destroy_node()
    rclpy.shutdown()