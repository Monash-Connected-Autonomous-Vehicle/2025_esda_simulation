#!/usr/bin/env python3

import math
import numpy as np

import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import msg
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from sensor_msgs.msg import LaserScan

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import subprocess


# Define the FollowTheGap class, which will implement the "Follow the Gap" algorithm for navigation
# This algorithm does not seek out the longest gap, but rather the gap that is closest to the goal direction, which can be more efficient in certain scenarios.
# Approach uses the Disparity Extender method to identify gaps in the occupancy grid and navigate through them towards the goal.
# The algorithm will be implemented as a ROS2 node that subscribes to the occupancy grid and publishes navigation goals to the Nav2 stack.
# Dispariities are gaps in the LiDAR scan data where 2 numbers next to each other differ majorly, indicating a potential gap in the environment. The algorithm will identify these gaps and evaluate them based on their alignment with the goal direction to determine the best path forward.

# 1. Find disparities in lidar readings
# 2. For each disparity, extend it half the width of the robot to find the gap
# 3. Evaluate the gap based on its alignment with the goal direction
# 4. Select the best gap and navigate towards it

# Making sure that car doesn't hit a corner
# 1. Scan all available LiDAR samples below -90 degrees and above 90 degrees
# 2. If any of these samples are below a certain threshold, consider the path blocked... or if any point is below safe distance on side of car in the direction the car is going, stop turning and keep going straight
# 3. If the path is blocked, stop the robot and re-evaluate the environment

# Wigglling problem - Robot keeps turning left and right --> 'S' shape --> Set limit threshold e,g, 2m or 3m, then follow centre of deepest gap until the robot is within the threshold distance to the goal, then switch to a different navigation strategy (e.g., A* or Dijkstra's) to navigate the remaining distance to the goal.

def load_robot_xml(filepath):
    result = subprocess.run(
        ['xacro', filepath],
        capture_output=True,
        text=True
    )
    return result.stdout

def extract_param_from_xml(xml_text, property_name='chassis_width'):
    root = ET.fromstring(xml_text)

    # ---- 1. Try to find property (ONLY works if xacro NOT expanded) ----
    for elem in root.iter():
        tag = elem.tag.split('}')[-1]

        if tag == 'property' and elem.attrib.get('name') == property_name:
            try:
                return float(elem.attrib.get('value'))
            except (TypeError, ValueError):
                pass

    # ---- 2. Fallback: extract from geometry (expanded XML) ----
    for elem in root.iter():
        tag = elem.tag.split('}')[-1]

        if tag == 'box':
            size = elem.attrib.get('size', '')
            parts = size.split()

            if len(parts) == 3:
                try:
                    # x, y, z → width is y
                    return float(parts[1])
                except ValueError:
                    pass

    return None

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')
        
        package_share = Path(get_package_share_directory('esda_simulation_2025'))
        robot_description_file = package_share / 'description' / 'robot_core_ref.xacro'

        self.declare_parameter('robot_description_file', str(robot_description_file))

        # Declare parameters related to the map and navigation
        self.declare_parameter('goal_pose', [0.0, 0.0, 0.0])  # [x, y, theta]
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')

        # Getting the parameters from the parameter server
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.goal_pose = self.get_parameter('goal_pose').get_parameter_value().double_array_value

        # Declare parameters related to the Follow the Gap algorithm
        self.declare_parameter('lidar_topic', '/scan')
        robot_file = self.get_parameter('robot_description_file').get_parameter_value().string_value
        robot_xml = load_robot_xml(robot_file)
        self.robot_x_width = extract_param_from_xml(robot_xml, 'chassis_width') # Chassis width
        self.robot_y_width = extract_param_from_xml(robot_xml, 'chassis_length') # Chassis length
        self.disparity_threshold = 0.5  # Threshold for identifying disparities in LiDAR data (in meters)

        self.robot_radius = self.robot_x_width / 2.0  # Assuming the robot's width is the limiting factor for navigation
        self.ftg_safety_radius = self.robot_radius + 0.1  # Adding a safety margin to the robot's radius ~ 0.35 meters total

        # Declare parameters related to the Disparity Extender method
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Declaring the topic to subscribe to: navigate to pose
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value

        # Creating the subscriber for the occupancy grid
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info('FollowTheGap node has been initialized with the following parameters:')
        self.get_logger().info(f"Robot Description File: {self.get_parameter('robot_description_file').get_parameter_value().string_value}")
        self.get_logger().info(f"Robot Width: {self.robot_x_width}")
        self.get_logger().info(f"Robot Length: {self.robot_y_width}")
        self.get_logger().info(f"Robot Radius: {self.robot_radius}")
        self.get_logger().info(f"FTG Safety Radius: {self.ftg_safety_radius}")

        pass
    
    def find_disparities(self, lidar_data):
        disparities = []

        # Need at least 2 points to compare neighbours
        if len(lidar_data) < 2:
            return disparities

        for i in range(1, len(lidar_data)):
            left = float(lidar_data[i - 1])
            right = float(lidar_data[i])

            if abs(right - left) > self.disparity_threshold:
                disparities.append({
                    'left_index': i - 1,
                    'right_index': i,
                    'left_distance': left,
                    'right_distance': right,
                    'closer_index': i - 1 if left < right else i,
                    'closer_distance': min(left, right),
                })

        return disparities
    
    def extend_disparity(self, forward_ranges, disparities, angle_increment):
        extended_ranges = np.array(forward_ranges, copy=True)

        if len(extended_ranges) == 0 or len(disparities) == 0:
            return extended_ranges

        for disparity in disparities:
            closer_index = disparity['closer_index']
            closer_distance = disparity['closer_distance']
            left_distance = disparity['left_distance']
            right_distance = disparity['right_distance']

            # Avoid divide-by-zero / nonsense values
            if closer_distance <= 0.0 or angle_increment <= 0.0:
                continue

            # Convert safety radius (m) into angular width, then into number of scan points
            points_to_extend = int(
                math.ceil((self.ftg_safety_radius / closer_distance) / angle_increment)
            )

            # Decide which way to extend:
            # if left side is closer obstacle, extend to the left
            # if right side is closer obstacle, extend to the right
            if left_distance < right_distance:
                start = max(0, closer_index - points_to_extend)
                end = closer_index + 1
            else:
                start = closer_index
                end = min(len(extended_ranges), closer_index + points_to_extend + 1)

            # Mark those neighbouring points as equally unsafe
            extended_ranges[start:end] = np.minimum(
                extended_ranges[start:end],
                closer_distance
            )

        return extended_ranges

    def scan_callback(self, msg):
        # This callback will be triggered whenever a new LiDAR scan is received
        # The msg parameter will contain the LaserScan data, which can be processed to find disparities and navigate towards the goal
        
        # 1. Convert ranges to numpy array for easier processing
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 2. Guard against empty scans
        if ranges.size == 0:
            self.get_logger().warn("Received empty LaserScan")
            return

        # 3. Replace invalid values
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max

        # 4. Clip to valid sensor range
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        # 5. Compute angle for each scan point
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        # 6. Keep only forward-facing scan points
        forward_mask = (angles >= -math.pi / 2.0) & (angles <= math.pi / 2.0)
        forward_ranges = ranges[forward_mask]
        forward_angles = angles[forward_mask]

        if forward_ranges.size == 0:
            self.get_logger().warn("No forward-facing LiDAR points available")
            return

        # 7. Find disparities in the forward-facing LiDAR data
        disparities = self.find_disparities(forward_ranges)

        # 8. Basic debug information
        min_idx = int(np.argmin(forward_ranges))
        min_range = float(forward_ranges[min_idx])
        min_angle = float(forward_angles[min_idx])

        # 9. Extend the disparities to find potential gaps
        extended_gaps = self.extend_disparity(forward_ranges, disparities, msg.angle_increment)

        pass


def main():
    # Launches the FollowTheGap node and keeps it running until shutdown
    rclpy.init()
    node = FollowTheGap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()