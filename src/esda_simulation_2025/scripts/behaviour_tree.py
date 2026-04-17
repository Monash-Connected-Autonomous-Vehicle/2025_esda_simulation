#!/usr/bin/env python3

# This node will implement a behaviour tree to manage the different navigation strategies for the robot. It will subscribe to the necessary topics to determine the current state of the robot and the environment, and will switch between different navigation strategies (e.g. follow the gap, goal navigation) based on the current state and the distance to the goal.
# The behaviour tree will be structured as follows:
# - Cases with lanes:
#   - Centreline following: If the robot detects lane markings and is not close to the goal, it will follow the centreline of the track using a lane following algorithm.
#   - LiDAR says clear --> Robot follows centreline
# - Cases where obstacle appears in front of the robot:
#   - Follow the gap: If the robot detects an obstacle in front of it, it will switch to the follow the gap algorithm to navigate around the obstacle until it is clear again, at which point it will switch back to centreline following.
#   - LiDAR says blocked --> Robot follows the gap
# - Cases where no lane is detected --> No centreline following, robot follows the gap
#   - Fallback to FTG: If the robot does not detect any lane markings, it will default to using the follow the gap algorithm to navigate until it detects lanes again or gets close to the goal.
# - Cases where the robot is close to the goal:
#   - Goal navigation: If the robot is within a certain distance to the goal, it will switch to a goal navigation strategy (e.g. using the Nav2 stack) to navigate directly towards the goal.
#   - Distance to goal < threshold --> Robot switches to goal navigation

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

import rclpy
from rclpy.node import Node

class BehaviourTree(Node):
    def __init__(self):
        super().__init__('behaviour_tree')
    
    def lane_callback(self, msg):
        # Process lane detection data from the MarkerArray message to determine if lanes are detected and to assist in determining track direction or goal location if needed.
        markers = msg.markers
        # Implement logic to determine if lanes are detected and to extract relevant information for navigation decisions.

    def lidar_callback(self, msg):
        # Process LiDAR data from the LaserScan message to determine if there are obstacles in front of the robot and to assist in determining track direction or goal location if needed.
        ranges = msg.ranges
        # Implement logic to determine if there are obstacles in front of the robot and to extract relevant information for navigation decisions.

    
    