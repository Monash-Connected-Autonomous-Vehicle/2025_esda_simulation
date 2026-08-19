#!/usr/bin/env python3

"""
This is a ROS2 Node that returns if whether a curve has been detected in front of the robot

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CurveDetectionNode(Node):
    def __init__(self):
        super().__init__('curve_detection')

        # 1. Lookahead distance parameters
        self.declare_parameter('curve_lookahead_distant', 2.0) # Lookahead distance in metres
        self.declare_parameter('curve_width', 2.0)

        # Getting the distance parameters
        self.curve_lookahead_distant = (
            self.get_parameter('curve_lookahead_distant'),
            .get_parameter_value()
            .double_value
        )

        self.curve_width = (
            self.get_parameter('curve_width')
            .get_parameter_value()
            .double_value
        )

        # TF parameters
        self.tf_buffer = Buffer()

        # Subscribe to the Occupancy Grid
        self.occupancy_grid_subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.occupancy_listener_callback,
            10
        )



        self.get_logger().info("Curve Detection Node started")

    def occupancy_listener_callback(self, msg):
        # Callback function that executes whenever a scan of the Occupancy Grid occurs
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid = np.array(msg.data).reshape((height, width))

        occupied_cells = np.argwhere(grid >= 50)

        for row, col in occupied_cells:
            x = origin_x + (col + 0.5) * resolution
            y = origin_y + (row + 0.5) * resolution

            print(f"Obstacle at map x={x:.2f}, y={y:.2f}")

        self.get_logger().info(f'Received map: {width}x{height} at {resolution} m/cell')


if __name__ == '__main__':
    rclpy.init()
    curve_detector = CurveDetectionNode()

    # navigator.send_goal(navigator.goal_pose)  # Send the goal to the navigation stack
    rclpy.spin(curve_detector)
    rclpy.destroy_node(curve_detector)
    rclpy.shutdown()
