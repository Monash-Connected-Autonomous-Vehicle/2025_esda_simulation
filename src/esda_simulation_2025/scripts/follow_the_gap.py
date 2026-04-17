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
        super().__init__('follow_the_gap')
        
        package_share = Path(get_package_share_directory('esda_simulation_2025'))
        robot_description_file = package_share / 'description' / 'robot_core_ref.xacro'

        self.declare_parameter('robot_description_file', str(robot_description_file))

        # Declare parameters related to the map and navigation
        self.declare_parameter('goal_pose', [10.0, 3.0, 0.0])  # [x, y, theta]
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')

        # Need to get the robot's pose from TF, so we set up a TF listener to get the robot's current position and orientation in the map frame. This will allow us to calculate the angle to the goal and evaluate the alignment of the identified gaps with the goal direction.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        self.safe_distance = 2.5  # Minimum safe distance to obstacles (in meters). Used after extending disparities to determine if a gap is navigable.

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

        # Parameters for sending goals to Nav2
        self.last_goal = None
        self.last_goal_time = self.get_clock().now()
        self.goal_update_period = 1.0  # seconds
        self.min_goal_shift = 0.5      # metres
        self.min_yaw_shift = 0.35      # radians

        self.get_logger().info('FollowTheGap node has been initialized with the following parameters:')
        self.get_logger().info(f"Robot Description File: {self.get_parameter('robot_description_file').get_parameter_value().string_value}")
        self.get_logger().info(f"Robot Width: {self.robot_x_width}")
        self.get_logger().info(f"Robot Length: {self.robot_y_width}")
        self.get_logger().info(f"Robot Radius: {self.robot_radius}")
        self.get_logger().info(f"FTG Safety Radius: {self.ftg_safety_radius}")

    
    
    def send_navigation_goal(self, target_x, target_y, target_theta):
        # This function sends a navigation goal to the Nav2 stack to navigate towards the specified target position and orientation. It constructs a NavigateToPose action goal with the target pose and sends it to the action server, allowing the robot to navigate towards the desired location.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0

        # Convert target_theta (yaw) to quaternion
        qz = math.sin(target_theta / 2.0)
        qw = math.cos(target_theta / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)

    def score_gap(self, extended_ranges, safe_groups, safe_masks, forward_angles, goal_angle):
        # This function takes in the extended ranges, safe groups, safe masks and goal angle, and evaluates the identified gaps based on their alignment with the goal direction. It calculates a score for each gap based on how closely it aligns with the goal angle, and returns the best gap to navigate towards.
        best_score = float('inf')
        best_gap = None

        for (is_safe, indices) in safe_groups:
            if not is_safe:
                continue

            gap_angles = forward_angles[indices]
            gap_center_angle = np.mean(gap_angles)

            # Calculate the score based on the absolute difference between the gap center angle and the goal angle
            score = abs(wrap_to_pi(gap_center_angle - goal_angle))

            if score < best_score:
                best_score = score
                best_gap = (gap_center_angle, indices)

        return best_gap

    def find_disparities(self, lidar_data):
        # This function finds the disparities in the LiDAR data, which are points where the distance readings change significantly, indicating a potential gap in the environment.

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
        # This function takes the identified disparities and extends them by marking neighboring points as unsafe based on the robot's safety radius. This helps to identify the full extent of the gap and ensures that the robot does not attempt to navigate through a space that is too narrow.
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
        
        # Get the robot transform to determine the robot's current position and orientation in the map frame. This is necessary for calculating the angle to the goal and evaluating the alignment of the identified gaps with the goal direction.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            # Convert quaternion to yaw angle
            q = transform.transform.rotation
            self.theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        
        except TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return

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
        print(f"Disparities: {disparities}")


        # 8. Basic debug information
        min_idx = int(np.argmin(forward_ranges))
        min_range = float(forward_ranges[min_idx])
        min_angle = float(forward_angles[min_idx])

        # 9. Extend the disparities to find potential gaps
        extended_ranges = self.extend_disparity(forward_ranges, disparities, msg.angle_increment)
        print(f"Extended ranges: {extended_ranges}")


        # 10. Find gaps gaps if whether the extended ranges are greater than the safe distance, and evaluate them based on their alignment with the goal direction to determine the best path forward. This part of the implementation will involve calculating the angle to the goal and comparing it with the angles of the identified gaps to select the most suitable one for navigation.
        safe_mask = extended_ranges >= self.safe_distance
        safe_angles = extended_ranges[safe_mask]
        safe_ranges = extended_ranges[safe_mask]

        # 11. Group the gaps together
        safe_groups = [
            (key, [idx for idx, val in group])
            for key, group in groupby(enumerate(safe_mask), key = lambda x: x[1])

        ]
        
        print(f"Safe groups: {safe_groups}")

        # 12. Evaluate the gaps based on their alignment with the goal direction to determine the best path forward. This will involve calculating the angle to the goal and comparing it with the angles of the identified gaps to select the most suitable one for navigation.
        goal_angle_world = math.atan2(self.goal_pose[1] - self.y, self.goal_pose[0] - self.x)

        # convert world goal direction into robot-relative direction
        goal_angle_robot = wrap_to_pi(goal_angle_world - self.theta)

        best_gap = self.score_gap(extended_ranges, safe_groups, safe_mask, forward_angles, goal_angle_robot)

        print(f"Best gap: {best_gap}")

        # 13. If a suitable gap is found, publish a navigation goal towards the center of that gap. This will involve calculating the target position based on the angle and distance of the best gap, and sending a goal to the Nav2 stack to navigate towards that position. 
        if best_gap is None:
            self.get_logger().warn("No safe gap found")
            return

        gap_center_angle, gap_indices = best_gap

        # 🔥 Add this line here
        gap_center_angle *= 0.5  # reduce turning aggressiveness

        gap_mid_idx = gap_indices[len(gap_indices) // 2]
        # gap_distance = float(extended_ranges[gap_mid_idx])
        gap_distance = min(float(extended_ranges[gap_mid_idx]), 1.5)

        orientation = wrap_to_pi(self.theta + gap_center_angle)
        target_x = self.x + gap_distance * math.cos(orientation)
        target_y = self.y + gap_distance * math.sin(orientation)

        now = self.get_clock().now()
        time_since_last = (now - self.last_goal_time).nanoseconds / 1e9

        send_new_goal = False

        if self.last_goal is None:
            send_new_goal = True
        else:
            last_x, last_y, last_yaw = self.last_goal
            dist_shift = math.hypot(target_x - last_x, target_y - last_y)
            yaw_shift = abs(wrap_to_pi(orientation - last_yaw))

            if time_since_last >= self.goal_update_period and (
                dist_shift > self.min_goal_shift or yaw_shift > self.min_yaw_shift
            ):
                send_new_goal = True

        if send_new_goal:
            self.send_navigation_goal(target_x, target_y, orientation)
            self.last_goal = (target_x, target_y, orientation)
            self.last_goal_time = now

        # orientation = self.theta
        # self.send_navigation_goal(target_x, target_y, orientation)

        print(f"Target position: ({target_x:.2f}, {target_y:.2f}), orientation: {math.degrees(orientation):.1f} degrees")

        


def main():
    # Launches the FollowTheGap node and keeps it running until shutdown
    rclpy.init()
    node = FollowTheGap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()