#!/usr/bin/env python3
import math
import time

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

        self.declare_parameter('loop_forever', True)
        self.declare_parameter('pause_seconds', 2.0)

        self.loop_forever = self.get_parameter('loop_forever').value
        self.pause_seconds = self.get_parameter('pause_seconds').value

        self.get_logger().info(f'Loop forever: {self.loop_forever}')
        self.get_logger().info(f'Pause seconds: {self.pause_seconds}')

        # Start once ROS is spinning
        self.start_timer = self.create_timer(1.0, self.start_once)
        self.started = False

    def start_once(self):
        if self.started:
            return
        self.started = True
        self.start_timer.cancel()
        self.send_waypoints()

    def load_waypoints(self):
        return [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi / 2),
            (0.0, 1.0, math.pi),
            (0.0, 0.0, -math.pi / 2),
        ]

    def build_goal(self):
        goal_msg = FollowWaypoints.Goal()

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

    def result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints

        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
        else:
            self.get_logger().info('Completed all waypoints')

        if self.loop_forever:
            self.get_logger().info(f'Waiting {self.pause_seconds} seconds before restarting...')
            time.sleep(self.pause_seconds)
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