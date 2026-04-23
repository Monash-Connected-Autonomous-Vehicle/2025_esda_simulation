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

# Ultimately, this code basically combines the lane following and follow the gap algorithms into a single node that can switch between the two based on the current state of the robot and the environment. It also incorporates a goal navigation strategy for when the robot is close to the goal, allowing it to navigate directly towards the goal when appropriate.
# FTG for close objects
# track_follower for when lanes are detected and no close obstacles for general palnner

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

import rclpy
from rclpy.node import Node

from enum import Enum

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
from esda_simulation_2025.msg import NavigationRecommendation

# Define an enumeration for the different navigation states of the robot. This will help us manage the different navigation strategies in a clear and organized way, allowing us to easily switch between them based on the current state of the robot and the environment.
class NavigationState(Enum):
    CENTRELINE_FOLLOWING = 1
    FOLLOW_THE_GAP = 2
    GOAL_NAVIGATION = 3
    RECOVERY = 4  # Added a recovery state for handling situations where the robot is stuck or encounters an unexpected situation


class BehaviourTree(Node):
    def __init__(self):
        super().__init__('behaviour_tree')

        # Subscribing to different topics
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')   

        # Subscribes to the track follower node
        self.track_follower_subscription = self.create_subscription(
            Twist, 
            '/track_follower', 
            self.track_follower_callback, 
            10
        )

        # Subscribes to the follow the gap node
        self.follow_the_gap_subscription = self.create_subscription(
            NavigationRecommendation, 
            '/follow_the_gap_cmd_vel', 
            self.follow_the_gap_callback, 
            10
        )

        # Start the robot in the centreline following state by default. We will switch to other states based on the sensor data and the distance to the goal.
        self.current_state = NavigationState.CENTRELINE_FOLLOWING


        # Defining the timer control loop period
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)

    
    
    def control_loop(self):
        # This is the main control loop that will be called periodically by the timer. In this loop, we will check the current state of the robot and the environment based on the latest sensor data, and we will switch between different navigation strategies accordingly. We will also implement a recovery strategy for handling situations where the robot is stuck or encounters an unexpected situation.
        if self.current_state == NavigationState.CENTRELINE_FOLLOWING:
            # Implement logic for centreline following navigation strategy

            pass
        elif self.current_state == NavigationState.FOLLOW_THE_GAP:
            # Implement logic for follow the gap navigation strategy
            pass
        elif self.current_state == NavigationState.GOAL_NAVIGATION:
            # Implement logic for goal navigation strategy
            pass
        elif self.current_state == NavigationState.RECOVERY:
            # Implement logic for recovery strategy to handle situations where the robot is stuck or encounters an unexpected situation
            pass

    def track_follower_callback(self, msg):
        # This callback will be called whenever a new message is received from the track follower node. We will use this information to update our current state and make decisions about which navigation strategy to use.
        self.get_logger().info(
            f'Received Track Follower cmd_vel: linear_x={msg.linear.x:.2f} m/s, angular_z={msg.angular.z:.2f} rad/s'
        )

    def follow_the_gap_callback(self, msg):
        # This callback will be called whenever a new message is received from the follow the gap node. We will use this information to update our current state and make decisions about which navigation strategy to use.
        self.get_logger().info(
            f'Received Follow the Gap recommendation: valid={msg.valid}, confidence={msg.confidence:.2f}, linear_x={msg.linear_x:.2f}, angular_z={msg.angular_z:.2f}'
        )

def main():
    rclpy.init()
    behaviour_tree_node = BehaviourTree()
    rclpy.spin(behaviour_tree_node)
    behaviour_tree_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()