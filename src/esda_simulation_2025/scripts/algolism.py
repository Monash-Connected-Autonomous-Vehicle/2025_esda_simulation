#!/usr/bin/env python3

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/diff_drive_base_controller/odom')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('planning_period', 1.0)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('blacklist_radius', 0.75)
        self.declare_parameter('wall_inflation_radius_cells', 1)
        self.declare_parameter('goal_yaw_mode', 'face_goal')  # face_goal or fixed
        self.declare_parameter('fixed_goal_yaw', 0.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.navigate_action = self.get_parameter('navigate_action').value
        self.planning_period = float(self.get_parameter('planning_period').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.wall_inflation_radius_cells = int(
            self.get_parameter('wall_inflation_radius_cells').value
        )
        self.goal_yaw_mode = str(self.get_parameter('goal_yaw_mode').value)
        self.fixed_goal_yaw = float(self.get_parameter('fixed_goal_yaw').value)

        self.stop_immediate_retries_test_param = True

        # State
        self.map_array = None
        self.map_info = None
        self.robot_x = None
        self.robot_y = None

        # Remembers the last goal   
        self.last_reached_goal = None
        self.reached_goal_radius = 0.15

        self.robot_yaw = 0.0

        self.goal_active = False
        self.goal_handle = None
        self.current_goal = None
        self.goal_sent_time = None
        self.blacklist = []

        # ROS interfaces
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)

        self.timer = self.create_timer(self.planning_period, self.timer_callback)

        self.get_logger().info('Proper frontier exploration node started.')

    def is_near_last_reached_goal(self, wx, wy):
        if self.last_reached_goal is None:
            return False

        lx, ly = self.last_reached_goal
        return math.hypot(wx - lx, wy - ly) < self.reached_goal_radius

    def map_callback(self, msg: OccupancyGrid):
        self.map_info = msg.info
        self.map_array = np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def timer_callback(self):
        if self.map_array is None or self.map_info is None:
            self.get_logger().info('Waiting for map...')
            return

        if self.robot_x is None or self.robot_y is None:
            self.get_logger().info('Waiting for odom...')
            return

        if self.goal_active:
            self.get_logger().info('Goal already active...')
            return

        clusters = self.detect_frontier_clusters()
        self.get_logger().info(f'Detected {len(clusters)} frontier clusters')

        if not clusters:
            self.get_logger().info('No valid frontiers found.')
            return

        goal = self.select_best_goal(clusters)
        self.get_logger().info(f'Selected goal: {goal}')

        if goal is None:
            self.get_logger().info('All frontiers invalid or blacklisted.')
            return

    

        self.send_navigation_goal(goal)

    def detect_frontier_clusters(self):
        """
        Frontier definition:
        - current cell is FREE (0)
        - at least one neighbour is UNKNOWN (-1)
        Then cluster neighbouring frontier cells.
        """
        h, w = self.map_array.shape
        frontier_mask = np.zeros((h, w), dtype=bool)

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if self.map_array[y, x] != 0:
                    continue

                if self.is_near_wall(x, y, self.wall_inflation_radius_cells):
                    continue

                neighbors = self.map_array[y - 1:y + 2, x - 1:x + 2]
                if np.any(neighbors == -1):
                    frontier_mask[y, x] = True

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        for y in range(h):
            for x in range(w):
                if not frontier_mask[y, x] or visited[y, x]:
                    continue

                cluster = []
                q = deque([(x, y)])
                visited[y, x] = True

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy))

                    for nx, ny in self.get_neighbors8(cx, cy, w, h):
                        if frontier_mask[ny, nx] and not visited[ny, nx]:
                            visited[ny, nx] = True
                            q.append((nx, ny))

                if len(cluster) >= self.min_frontier_size:
                    clusters.append(cluster)

        return clusters

    def get_neighbors8(self, x, y, width, height):
        for ny in range(max(0, y - 1), min(height, y + 2)):
            for nx in range(max(0, x - 1), min(width, x + 2)):
                if nx == x and ny == y:
                    continue
                yield nx, ny

    def is_near_wall(self, x, y, threshold_cells):
        h, w = self.map_array.shape
        for dy in range(-threshold_cells, threshold_cells + 1):
            for dx in range(-threshold_cells, threshold_cells + 1):
                nx = x + dx
                ny = y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if self.map_array[ny, nx] >= 50:
                        return True
        return False

    def grid_to_world(self, gx, gy):
        wx = gx * self.map_info.resolution + self.map_info.origin.position.x
        wy = gy * self.map_info.resolution + self.map_info.origin.position.y
        return wx, wy

    def cluster_centroid_world(self, cluster):
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        cx = int(round(sum(xs) / len(xs)))
        cy = int(round(sum(ys) / len(ys)))
        return self.grid_to_world(cx, cy)
    
    def cluster_farthest_world(self, cluster):
        best = None
        best_dist = -1.0

        for gx, gy in cluster:
            wx, wy = self.grid_to_world(gx, gy)
            d = math.hypot(wx - self.robot_x, wy - self.robot_y)
            if d > best_dist:
                best_dist = d
                best = (wx, wy)

        return best

    def select_best_goal(self, clusters):
        """
        Prefer larger and closer frontiers.
        score = size - 2 * distance
        """
        best_goal = None
        best_score = float('-inf')

        for cluster in clusters:
            # wx, wy = self.cluster_centroid_world(cluster)
            wx, wy = self.cluster_farthest_world(cluster)

            if self.is_blacklisted(wx, wy):
                continue

            distance = math.hypot(wx - self.robot_x, wy - self.robot_y)

            if distance < 0.5:
                continue

            if self.is_near_last_reached_goal(wx, wy):
                continue

            # if distance < 0:
            #     continue

            size = len(cluster)
            score = size + 2.0 * distance

            if score > best_score:
                best_score = score
                best_goal = (wx, wy)

            self.get_logger().info(
            f"goal=({wx:.2f},{wy:.2f}) dist={distance:.2f} "
            f"blacklisted={self.is_blacklisted(wx, wy)} "
            f"near_last={self.is_near_last_reached_goal(wx, wy)}"
            )

        return best_goal

    def is_blacklisted(self, wx, wy):
        for bx, by in self.blacklist:
            if math.hypot(wx - bx, wy - by) < self.blacklist_radius:
                return True
        return False

    def blacklist_goal(self, goal):
        if goal is None:
            return
        self.blacklist.append(goal)
        self.get_logger().info(
            f'Blacklisted goal near ({goal[0]:.2f}, {goal[1]:.2f})'
        )

    def send_navigation_goal(self, goal):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('NavigateToPose action server not available.')
            return

        gx, gy = goal

        # if self.goal_yaw_mode == 'face_goal':
        #     yaw = math.atan2(gy - self.robot_y, gx - self.robot_x)
        # else:
        #     yaw = self.fixed_goal_yaw

        yaw = self.robot_yaw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending frontier goal: ({gx:.2f}, {gy:.2f})')

        self.goal_active = True
        self.current_goal = goal
        self.goal_sent_time = self.get_clock().now()

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Frontier goal was rejected.')
            self.blacklist_goal(self.current_goal)
            self.goal_active = False
            self.goal_handle = None
            self.current_goal = None
            self.goal_sent_time = None
            return

        self.goal_handle = goal_handle
        self.get_logger().info('Frontier goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.goal_active = False

        try:
            result_msg = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to get goal result: {e}')
            self.blacklist_goal(self.current_goal)
            self.goal_handle = None
            self.current_goal = None
            self.goal_sent_time = None
            return

        status = result_msg.status

        # action_msgs/GoalStatus:
        # 4 = STATUS_SUCCEEDED
        if status == 4:
            self.get_logger().info('Frontier goal succeeded.')
            self.last_reached_goal = self.current_goal
            # self.blacklist_goal(self.current_goal)
        else:
            # self.get_logger().warn(f'Frontier goal failed with status {status}.')
            # self.blacklist_goal(self.current_goal)
            # self.get_logger().info(f'Blacklisted goal at ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})')
            self.get_logger().warn(f'Frontier goal failed with status {status}.')

        self.goal_handle = None
        self.current_goal = None
        self.goal_sent_time = None

        # Try immediately again for more continuous exploration

        if self.stop_immediate_retries_test_param:
            clusters = self.detect_frontier_clusters()
            if clusters:
                next_goal = self.select_best_goal(clusters)
                if next_goal is not None:
                    self.send_navigation_goal(next_goal)

    def cancel_current_goal(self):
        if self.goal_handle is None:
            self.goal_active = False
            self.current_goal = None
            self.goal_sent_time = None
            return

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        self.get_logger().info('Cancelled current frontier goal.')
        self.goal_active = False
        self.goal_handle = None
        self.current_goal = None
        self.goal_sent_time = None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()