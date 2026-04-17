#!/usr/bin/env python3

import math
import numpy as np
import xml.etree.ElementTree as ET
import subprocess

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

from itertools import groupby
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def load_robot_xml(filepath):
    result = subprocess.run(
        ['xacro', filepath],
        capture_output=True,
        text=True
    )
    return result.stdout


def extract_param_from_xml(xml_text, property_name='chassis_width'):
    root = ET.fromstring(xml_text)

    for elem in root.iter():
        tag = elem.tag.split('}')[-1]
        if tag == 'property' and elem.attrib.get('name') == property_name:
            try:
                return float(elem.attrib.get('value'))
            except (TypeError, ValueError):
                pass

    for elem in root.iter():
        tag = elem.tag.split('}')[-1]
        if tag == 'box':
            size = elem.attrib.get('size', '')
            parts = size.split()
            if len(parts) == 3:
                try:
                    return float(parts[1])  # y = width
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
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('lane_marker_topic', '/lane_segments')
        self.declare_parameter('occupancy_grid_topic', '/map')

        # FTG parameters
        self.declare_parameter('disparity_threshold', 0.5)
        self.declare_parameter('safe_distance', 1.2)
        self.declare_parameter('safety_margin', 0.1)
        self.declare_parameter('forward_fov_deg', 120.0)

        # Control parameters
        self.declare_parameter('max_linear_speed', 0.7)
        self.declare_parameter('min_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('steer_gain', 1.3)
        self.declare_parameter('turn_slowdown_angle', 0.8)
        self.declare_parameter('aggressiveness_scale', 0.5)

        # Lane-follow parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('lane_timeout_sec', 0.5)
        self.declare_parameter('max_lane_preferred_angle_deg', 35.0)
        self.declare_parameter('lane_blend_weight', 0.85)
        self.declare_parameter('max_gap_angle_deg', 20.0)

        # Occupancy-grid filtering parameters
        self.declare_parameter('grid_projection_distance', 0.8)
        self.declare_parameter('grid_unknown_is_blocked', True)
        self.declare_parameter('grid_occupied_threshold', 80)
        self.declare_parameter('grid_cost_penalty_scale', 2.0)
        self.declare_parameter('grid_check_half_width_m', 0.25)

        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.lane_marker_topic = self.get_parameter('lane_marker_topic').value
        self.occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').value

        robot_file = self.get_parameter('robot_description_file').value
        robot_xml = load_robot_xml(robot_file)

        self.robot_x_width = extract_param_from_xml(robot_xml, 'chassis_width')
        self.robot_y_width = extract_param_from_xml(robot_xml, 'chassis_length')

        if self.robot_x_width is None:
            self.get_logger().warn("Could not extract chassis_width, defaulting to 0.5 m")
            self.robot_x_width = 0.5

        if self.robot_y_width is None:
            self.get_logger().warn("Could not extract chassis_length, defaulting to 0.7 m")
            self.robot_y_width = 0.7

        self.disparity_threshold = float(self.get_parameter('disparity_threshold').value)
        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.forward_fov_rad = math.radians(float(self.get_parameter('forward_fov_deg').value) / 2.0)

        self.robot_radius = self.robot_x_width / 2.0
        self.ftg_safety_radius = self.robot_radius + self.safety_margin

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.steer_gain = float(self.get_parameter('steer_gain').value)
        self.turn_slowdown_angle = float(self.get_parameter('turn_slowdown_angle').value)
        self.aggressiveness_scale = float(self.get_parameter('aggressiveness_scale').value)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.lane_timeout_sec = float(self.get_parameter('lane_timeout_sec').value)
        self.max_lane_preferred_angle = math.radians(
            float(self.get_parameter('max_lane_preferred_angle_deg').value)
        )
        self.lane_blend_weight = float(self.get_parameter('lane_blend_weight').value)
        self.max_gap_angle = math.radians(float(self.get_parameter('max_gap_angle_deg').value))

        self.grid_projection_distance = float(self.get_parameter('grid_projection_distance').value)
        self.grid_unknown_is_blocked = bool(self.get_parameter('grid_unknown_is_blocked').value)
        self.grid_occupied_threshold = int(self.get_parameter('grid_occupied_threshold').value)
        self.grid_cost_penalty_scale = float(self.get_parameter('grid_cost_penalty_scale').value)
        self.grid_check_half_width_m = float(self.get_parameter('grid_check_half_width_m').value)

        self.latest_lane_angle = None
        self.latest_lane_confidence = 0.0
        self.last_lane_time = None

        self.latest_grid = None

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )

        self.lane_subscriber = self.create_subscription(
            MarkerArray,
            self.lane_marker_topic,
            self.lane_callback,
            10
        )

        self.grid_subscriber = self.create_subscription(
            OccupancyGrid,
            self.occupancy_grid_topic,
            self.grid_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info('FollowTheGap + lane guidance + occupancy grid node initialised')
        self.get_logger().info(f"LiDAR topic: {self.lidar_topic}")
        self.get_logger().info(f"Lane marker topic: {self.lane_marker_topic}")
        self.get_logger().info(f"Occupancy grid topic: {self.occupancy_grid_topic}")
        self.get_logger().info(f"cmd_vel topic: {self.cmd_vel_topic}")

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def grid_callback(self, msg: OccupancyGrid):
        self.latest_grid = msg

    def marker_to_xy_points(self, marker):
        pts = []
        for p in marker.points:
            pts.append((float(p.x), float(p.y)))
        return pts

    def infer_lane_side(self, marker, points):
        ns = marker.ns.lower() if marker.ns else ""
        if 'left' in ns:
            return 'left'
        if 'right' in ns:
            return 'right'

        if not points:
            return None

        mean_y = float(np.mean([p[1] for p in points]))
        if mean_y > 0.0:
            return 'left'
        if mean_y < 0.0:
            return 'right'
        return None

    def choose_point_near_lookahead(self, points, lookahead_distance):
        front_points = [(x, y) for (x, y) in points if x > 0.2]
        if not front_points:
            return None

        best_pt = min(front_points, key=lambda p: abs(p[0] - lookahead_distance))
        return best_pt

    def lane_callback(self, msg: MarkerArray):
        left_candidates = []
        right_candidates = []

        for marker in msg.markers:
            pts = self.marker_to_xy_points(marker)
            if len(pts) < 2:
                continue

            side = self.infer_lane_side(marker, pts)
            if side == 'left':
                left_candidates.extend(pts)
            elif side == 'right':
                right_candidates.extend(pts)

        if len(left_candidates) < 2 or len(right_candidates) < 2:
            self.latest_lane_angle = None
            self.latest_lane_confidence = 0.0
            return

        left_pt = self.choose_point_near_lookahead(left_candidates, self.lookahead_distance)
        right_pt = self.choose_point_near_lookahead(right_candidates, self.lookahead_distance)

        if left_pt is None or right_pt is None:
            self.latest_lane_angle = None
            self.latest_lane_confidence = 0.0
            return

        cx = 0.5 * (left_pt[0] + right_pt[0])
        cy = 0.5 * (left_pt[1] + right_pt[1])

        if cx <= 0.05:
            self.latest_lane_angle = None
            self.latest_lane_confidence = 0.0
            return

        lane_angle = math.atan2(cy, cx)
        lane_angle = max(-self.max_lane_preferred_angle, min(self.max_lane_preferred_angle, lane_angle))

        lane_width = abs(left_pt[1] - right_pt[1])
        confidence = 1.0
        if lane_width < self.robot_x_width * 0.8:
            confidence = 0.4
        elif lane_width < self.robot_x_width * 1.2:
            confidence = 0.7

        self.latest_lane_angle = lane_angle
        self.latest_lane_confidence = confidence
        self.last_lane_time = self.get_clock().now()

    def lane_is_fresh(self):
        if self.last_lane_time is None:
            return False
        age = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        return age <= self.lane_timeout_sec

    def get_preferred_angle(self):
        if self.lane_is_fresh() and self.latest_lane_angle is not None:
            w = max(0.0, min(1.0, self.lane_blend_weight * self.latest_lane_confidence))
            return w * self.latest_lane_angle
        return 0.0

    def world_to_grid(self, x_world, y_world):
        if self.latest_grid is None:
            return None

        info = self.latest_grid.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution

        gx = int((x_world - origin_x) / resolution)
        gy = int((y_world - origin_y) / resolution)

        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return None

        return gx, gy

    def get_grid_value(self, gx, gy):
        if self.latest_grid is None:
            return None

        info = self.latest_grid.info
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return None

        idx = gy * info.width + gx
        return int(self.latest_grid.data[idx])

    def evaluate_projected_gap_cell(self, gap_angle):
        """
        Project a short point ahead in robot-local coordinates and check a few nearby
        cells around it in the occupancy grid.

        Assumes the occupancy grid and lane/robot-local geometry are aligned enough
        for this to be meaningful. Best if your grid is effectively in the same
        frame you reason about driving in.
        """
        if self.latest_grid is None:
            return True, 0.0

        x_proj = self.grid_projection_distance * math.cos(gap_angle)
        y_proj = self.grid_projection_distance * math.sin(gap_angle)

        sample_offsets = [0.0, -self.grid_check_half_width_m, self.grid_check_half_width_m]
        worst_value = 0

        for lateral in sample_offsets:
            sx = x_proj
            sy = y_proj + lateral

            grid_xy = self.world_to_grid(sx, sy)
            if grid_xy is None:
                return False, float('inf')

            gx, gy = grid_xy
            cell_value = self.get_grid_value(gx, gy)

            if cell_value is None:
                return False, float('inf')

            if cell_value == -1 and self.grid_unknown_is_blocked:
                return False, float('inf')

            if cell_value >= self.grid_occupied_threshold:
                return False, float('inf')

            if cell_value > worst_value:
                worst_value = cell_value

        penalty = self.grid_cost_penalty_scale * (worst_value / 100.0)
        return True, penalty

    def score_gap(self, safe_groups, forward_angles, preferred_angle=0.0):
        best_score = float('inf')
        best_gap = None

        for (is_safe, indices) in safe_groups:
            if not is_safe or not indices:
                continue

            gap_angles = forward_angles[indices]
            gap_center_angle = float(np.mean(gap_angles))

            if abs(gap_center_angle) > self.max_gap_angle:
                continue

            gap_score = abs(wrap_to_pi(gap_center_angle - preferred_angle))

            grid_ok, grid_penalty = self.evaluate_projected_gap_cell(gap_center_angle)
            if not grid_ok:
                continue

            total_score = gap_score + grid_penalty

            if total_score < best_score:
                best_score = total_score
                best_gap = (gap_center_angle, indices)

        return best_gap

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

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        if ranges.size == 0:
            self.get_logger().warn("Received empty LaserScan")
            self.publish_stop()
            return

        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        forward_mask = (angles >= -self.forward_fov_rad) & (angles <= self.forward_fov_rad)
        forward_ranges = ranges[forward_mask]
        forward_angles = angles[forward_mask]

        if forward_ranges.size == 0:
            self.get_logger().warn("No forward-facing LiDAR points available")
            self.publish_stop()
            return

        disparities = self.find_disparities(forward_ranges)
        extended_ranges = self.extend_disparity(forward_ranges, disparities, msg.angle_increment)

        safe_mask = extended_ranges >= self.safe_distance

        safe_groups = [
            (key, [idx for idx, _ in group])
            for key, group in groupby(enumerate(safe_mask), key=lambda x: x[1])
        ]

        preferred_angle = self.get_preferred_angle()

        best_gap = self.score_gap(
            safe_groups=safe_groups,
            forward_angles=forward_angles,
            preferred_angle=preferred_angle
        )

        if best_gap is None:
            self.get_logger().warn("No safe gap found after occupancy-grid filtering")
            self.publish_stop()
            return

        gap_center_angle, gap_indices = best_gap
        gap_center_angle *= self.aggressiveness_scale

        gap_mid_idx = gap_indices[len(gap_indices) // 2]
        gap_distance = min(float(extended_ranges[gap_mid_idx]), 1.5)

        angular_z = self.steer_gain * gap_center_angle
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        if self.turn_slowdown_angle <= 0.0:
            speed_scale = 1.0
        else:
            speed_scale = max(0.0, 1.0 - abs(gap_center_angle) / self.turn_slowdown_angle)

        linear_x = self.min_linear_speed + (self.max_linear_speed - self.min_linear_speed) * speed_scale

        if gap_distance < 1.0:
            linear_x *= 0.6

        if not self.lane_is_fresh() or self.latest_lane_angle is None:
            linear_x *= 0.6

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        lane_deg = math.degrees(preferred_angle)
        gap_deg = math.degrees(gap_center_angle)
        self.get_logger().info(
            f"lane_pref={lane_deg:.1f} deg, "
            f"gap_angle={gap_deg:.1f} deg, "
            f"gap_dist={gap_distance:.2f} m, "
            f"v={linear_x:.2f}, w={angular_z:.2f}"
        )


def main():
    rclpy.init()
    node = FollowTheGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()