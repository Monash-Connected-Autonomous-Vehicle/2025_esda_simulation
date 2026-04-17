#!/usr/bin/env python3

import math
import numpy as np

import xml.etree.ElementTree as ET



import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist 
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import msg
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan

from itertools import groupby

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

# NOTE: This code is to be used in conjunction with track_follower.py. which will handle the overall track following around the entire map

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

def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap_cmd_vel')

        package_share = Path(get_package_share_directory('esda_simulation_2025'))
        robot_description_file = package_share / 'description' / 'robot_core_ref.xacro'

        self.declare_parameter('robot_description_file', str(robot_description_file))
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')

        # FTG tuning
        self.declare_parameter('disparity_threshold', 0.5)
        self.declare_parameter('safe_distance', 1.5)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('max_speed', 0.45)
        self.declare_parameter('min_speed', 0.05)
        self.declare_parameter('max_turn_rate', 1.5)
        self.declare_parameter('steering_gain', 1.2)
        self.declare_parameter('forward_bias_gain', 0.35)

        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_turn_rate = self.get_parameter('max_turn_rate').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.forward_bias_gain = self.get_parameter('forward_bias_gain').value

        robot_file = self.get_parameter('robot_description_file').get_parameter_value().string_value
        robot_xml = load_robot_xml(robot_file)

        self.robot_x_width = extract_param_from_xml(robot_xml, 'chassis_width')
        self.robot_y_width = extract_param_from_xml(robot_xml, 'chassis_length')

        if self.robot_x_width is None:
            self.robot_x_width = 0.5
            self.get_logger().warn('Could not read chassis_width from xacro, defaulting to 0.5 m')

        if self.robot_y_width is None:
            self.robot_y_width = 0.7
            self.get_logger().warn('Could not read chassis_length from xacro, defaulting to 0.7 m')

        self.robot_radius = self.robot_x_width / 2.0
        self.ftg_safety_radius = self.robot_radius + self.safety_margin

        self.latest_map = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )

        self.occupancy_grid_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.occupancy_grid_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('FollowTheGap cmd_vel node initialised')
        self.get_logger().info(f'Robot width: {self.robot_x_width:.3f} m')
        self.get_logger().info(f'Robot length: {self.robot_y_width:.3f} m')
        self.get_logger().info(f'Robot radius: {self.robot_radius:.3f} m')
        self.get_logger().info(f'FTG safety radius: {self.ftg_safety_radius:.3f} m')
        self.get_logger().info(f'Safe distance threshold: {self.safe_distance:.3f} m')

    def occupancy_grid_callback(self, msg):
        self.latest_map = msg

    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def find_disparities(self, lidar_data):
        disparities = []

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

            if closer_distance <= 0.0 or angle_increment <= 0.0:
                continue

            points_to_extend = int(
                math.ceil((self.ftg_safety_radius / closer_distance) / angle_increment)
            )

            if left_distance < right_distance:
                start = max(0, closer_index - points_to_extend)
                end = closer_index + 1
            else:
                start = closer_index
                end = min(len(extended_ranges), closer_index + points_to_extend + 1)

            extended_ranges[start:end] = np.minimum(
                extended_ranges[start:end],
                closer_distance
            )

        return extended_ranges

    def score_gap(self, extended_ranges, safe_groups, forward_angles):
        best_score = -float('inf')
        best_gap = None

        for is_safe, indices in safe_groups:
            if not is_safe or len(indices) == 0:
                continue

            gap_angles = forward_angles[indices]
            gap_center_angle = float(np.mean(gap_angles))

            gap_depth = float(np.mean(extended_ranges[indices]))
            gap_width = len(indices)

            # Prefer wider and deeper gaps, with a mild preference for forward motion
            score = (
                1.5 * gap_depth
                + 0.04 * gap_width
                - self.forward_bias_gain * abs(gap_center_angle)
            )

            if score > best_score:
                best_score = score
                best_gap = (gap_center_angle, indices, gap_depth)

        return best_gap

    def scan_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y

            q = transform.transform.rotation
            self.theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

        except TransformException as e:
            self.get_logger().warn(f'Could not get transform: {e}')
            self.stop_robot()
            return

        ranges = np.array(msg.ranges, dtype=np.float32)

        if ranges.size == 0:
            self.get_logger().warn('Received empty LaserScan')
            self.stop_robot()
            return

        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        # Forward 180 degrees
        forward_mask = (angles >= -math.pi / 2.0) & (angles <= math.pi / 2.0)
        forward_ranges = ranges[forward_mask]
        forward_angles = angles[forward_mask]

        if forward_ranges.size == 0:
            self.get_logger().warn('No forward-facing LiDAR points available')
            self.stop_robot()
            return

        disparities = self.find_disparities(forward_ranges)
        extended_ranges = self.extend_disparity(forward_ranges, disparities, msg.angle_increment)

        safe_mask = extended_ranges >= self.safe_distance

        safe_groups = [
            (key, [idx for idx, _ in group])
            for key, group in groupby(enumerate(safe_mask), key=lambda x: x[1])
        ]

        best_gap = self.score_gap(extended_ranges, safe_groups, forward_angles)

        if best_gap is None:
            self.get_logger().warn('No safe gap found')
            self.stop_robot()
            return

        gap_center_angle, gap_indices, gap_depth = best_gap

        gap_mid_idx = gap_indices[len(gap_indices) // 2]
        gap_distance = float(extended_ranges[gap_mid_idx])

        # Steering directly toward the chosen gap
        angular_z = self.steering_gain * gap_center_angle
        angular_z = max(-self.max_turn_rate, min(self.max_turn_rate, angular_z))

        # Base speed from clearance
        linear_x = min(self.max_speed, 0.18 * gap_distance)

        # Slow down for harder turns
        turn_scale = max(0.25, 1.0 - min(abs(gap_center_angle) / 1.2, 0.75))
        linear_x *= turn_scale

        if abs(gap_center_angle) > 0.35:
            linear_x = min(linear_x, 0.20)

        if abs(gap_center_angle) > 0.60:
            linear_x = min(linear_x, 0.12)

        # If obstacle is very close ahead, slow more
        centre_idx = len(forward_ranges) // 2
        ahead_window = forward_ranges[max(0, centre_idx - 10): min(len(forward_ranges), centre_idx + 11)]
        min_ahead = float(np.min(ahead_window)) if ahead_window.size > 0 else msg.range_max

        if min_ahead < 1.0:
            linear_x = min(linear_x, 0.25)
        if min_ahead < 0.6:
            linear_x = 0.0

        if gap_distance >= self.safe_distance:
            linear_x = max(self.min_speed, linear_x)

        self.publish_cmd(linear_x, angular_z)

        self.get_logger().info(
            f'gap_angle={math.degrees(gap_center_angle):.1f} deg, '
            f'gap_depth={gap_depth:.2f} m, '
            f'cmd=({linear_x:.2f} m/s, {angular_z:.2f} rad/s)'
        )


def main():
    rclpy.init()
    node = FollowTheGap()
    rclpy.spin(node)
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()