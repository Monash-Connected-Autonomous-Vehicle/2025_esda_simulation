#!/usr/bin/env python3
import math
import time
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class LoopWaypointsNode(Node):
    def __init__(self):
        super().__init__('loop_waypoints_node')

        self.waypoints = self.load_waypoints()
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Waypoint loop parameters
        self.declare_parameter('loop_forever', True)
        self.declare_parameter('pause_seconds', 2.0)
        self.declare_parameter('num_waypoints', 4)
        self.declare_parameter('max_retries', 3)  # Max retries for missed waypoints

        # Safety testing regions - Needs tuning based on your map and robot size
        self.declare_parameter('x_min', 0.0)
        self.declare_parameter('x_max', 2.0)
        self.declare_parameter('y_min', 0.0)
        self.declare_parameter('y_max', 2.0)


        # Returning the actual values stored in the declared parameters
        self.loop_forever = self.get_parameter('loop_forever').value
        self.pause_seconds = self.get_parameter('pause_seconds').value
        self.num_waypoints = self.get_parameter('num_waypoints').value

        # Relative waypoints steps: (dx, dy)
        # Each point is releative to the previous one, forming a square loop
        self.load_relative_waypoints()

        # Absolute starting position in the map
        self.start_x = 0.0
        self.start_y = 0.0

        # Returning the declared parameters for safety testing
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value

        # Convert relative points -> absolute map waypoints with yaw
        self.waypoints = self.relative_to_absolute_waypoints(
            self.relative_waypoints,
            start_x=self.start_x,
            start_y=self.start_y,
            loop=False
        )

        self.get_logger().info(f'Loop forever: {self.loop_forever}')
        self.get_logger().info(f'Pause seconds: {self.pause_seconds}')
        self.get_logger().info(f'Number of waypoints: {self.num_waypoints}')

        # Start once ROS is spinning
        self.retry_attempts = 0
        self.start_timer = self.create_timer(1.0, self.start_once)
        self.started = False

        

    def start_once(self):
        if self.started:
            return
        self.started = True
        self.start_timer.cancel()
        self.send_waypoints()

    def retry_waypoints(self):
        pass

    def load_relative_waypoints(self):
        # Define relative waypoints to form a square loop
        self.relative_waypoints = [
            (1.0, 0.0),  # Move 1m forward
            (0.0, 1.0),  # Move 1m left
            (-1.0, 0.0), # Move 1m backward
            (0.0, -1.0)  # Move 1m right
        ]

    def relative_to_absolute_waypoints(self, relative_points, start_x=0.0, start_y=0.0, loop=False):
        """
        Convert relative (dx, dy) steps into absolute (x, y, yaw) waypoints in map frame.
        Yaw is chosen to face the direction of travel for each segment.
        """
        absolute_points = []
        current_x = start_x
        current_y = start_y

        for dx, dy in relative_points:
            next_x = current_x + dx
            next_y = current_y + dy

            yaw = math.atan2(dy, dx) if (dx != 0.0 or dy != 0.0) else 0.0
            absolute_points.append((next_x, next_y, yaw))

            current_x = next_x
            current_y = next_y

        # Optional: if looping, make the last waypoint face back toward the first
        if loop and len(absolute_points) > 1:
            first_x, first_y, _ = absolute_points[0]
            last_x, last_y, _ = absolute_points[-1]
            final_yaw = math.atan2(first_y - last_y, first_x - last_x)
            absolute_points[-1] = (last_x, last_y, final_yaw)

        return absolute_points


    # Generates waypoints randomly within the defined safety region - Not used in current implementation but can be enabled for testing
    def generate_random_waypoint(self):
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        # yaw = random.uniform(-math.pi, math.pi)
        yaw = 0.0  # Keep yaw fixed for simplicity, can be randomized if needed
        return (x, y, yaw)
    
    # This function can be expanded to store generated waypoints in a list
    def store_generated_waypoints(self):
        # This function can be expanded to store generated waypoints in a file or database for later analysis
        waypoints_list = []

        for _ in range(self.num_waypoints):
            waypoint = self.generate_random_waypoint()
            waypoints_list.append(waypoint)
            self.get_logger().info(f'Generated waypoint: {waypoint}')
        
        return waypoints_list

    def load_waypoints(self):
        return [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi / 2),
            (0.0, 1.0, math.pi),
            (0.0, 0.0, -math.pi / 2),
        ]
    
    def add_headings_to_waypoints(self, points, loop=True):
        waypoints_with_yaw = []

        n = len(points)
        for i, (x, y) in enumerate(points):
            if i < n - 1:
                next_x, next_y = points[i + 1]
            elif loop and n > 1:
                next_x, next_y = points[0]
            else:
                # For a non-looping final waypoint, keep same heading as previous segment if possible
                if n > 1:
                    prev_x, prev_y = points[i - 1]
                    yaw = math.atan2(y - prev_y, x - prev_x)
                    waypoints_with_yaw.append((x, y, yaw))
                    continue
                else:
                    waypoints_with_yaw.append((x, y, 0.0))
                    continue

            yaw = math.atan2(next_y - y, next_x - x)
            waypoints_with_yaw.append((x, y, yaw))

        return waypoints_with_yaw

    def regenerate_waypoints(self):
        self.waypoints = self.store_generated_waypoints()
        self.get_logger().info(f'Regenerated {len(self.waypoints)} waypoint(s) for next loop')

    def build_goal(self):
        goal_msg = FollowWaypoints.Goal()

        # self.waypoints = self.store_generated_waypoints()  # Load waypoints from the function

        for x, y, yaw in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            goal_msg.poses.append(pose)

        return goal_msg

    def send_waypoints(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints action server not available')
            return

        goal_msg = self.build_goal()
        self.get_logger().info('Sending waypoint loop goal...')

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal was rejected')
            return

        self.get_logger().info('Waypoint goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def send_custom_waypoints(self, waypoints):
        goal_msg = FollowWaypoints.Goal()

        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = x
            pose.pose.position.y = y

            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            goal_msg.poses.append(pose)

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints

        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')

            # Retry only missed waypoints
            retry_waypoints = [self.waypoints[i] for i in missed]

            self.get_logger().info(f'Retrying missed waypoints: {retry_waypoints}')
            self.send_custom_waypoints(retry_waypoints)
            return
        
        
        self.get_logger().info('Completed all waypoints')

        if self.loop_forever:
            self.get_logger().info(f'Waiting {self.pause_seconds} seconds before restarting...')
            time.sleep(self.pause_seconds)

            self.regenerate_waypoints()  # Optionally regenerate waypoints for the next loop

            self.send_waypoints()
        else:
            self.get_logger().info('Loop finished once, shutting down.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LoopWaypointsNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()