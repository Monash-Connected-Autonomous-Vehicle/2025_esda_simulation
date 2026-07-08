#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from collections import deque
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Declaring parameters for the waypoint navigator
        self.declare_parameter('map_topic', '/map') # Subscribes to the map topic to get the occupancy grid
        self.declare_parameter('robot_frame', 'base_link') # Subscribes to the robot frame to get the robot's current pose
        self.declare_parameter('frame_id', 'map') # Subscribes to the map frame to get the map's current pose


        self.declare_parameter('free_cell_max_value', 20)
        self.declare_parameter('min_cluster_cells', 8)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('planner_period_sec', 1.0)
        self.declare_parameter('min_forward_progress_m', 0.15)
        self.declare_parameter('forward_score_weight', 8.0)
        self.declare_parameter('goal_republish_distance_m', 0.5)
        self.declare_parameter('startup_forward_distance_m', 1.5)
        self.declare_parameter('startup_forward_lateral_tolerance_m', 0.75)
        self.declare_parameter('general_forward_distance_m', 2.5)
        self.declare_parameter('general_forward_lateral_tolerance_m', 1.2)
        self.declare_parameter('startup_min_forward_distance_m', 0.4)
        self.declare_parameter('startup_goal_hold_sec', 3.0)
        self.declare_parameter('startup_goal_min_progress_m', 0.5)
        self.declare_parameter('goal_reached_distance_m', 0.35)
        self.declare_parameter('goal_replan_timeout_sec', 6.0)
        self.declare_parameter('local_planning_radius_m', 2.0)
        self.declare_parameter('map_border_margin_cells', 3)

        # Getting the parameter values
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        
        self.free_cell_max_value = self.get_parameter('free_cell_max_value').get_parameter_value().integer_value
        self.min_cluster_cells = self.get_parameter('min_cluster_cells').get_parameter_value().integer_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        planner_period_sec = self.get_parameter('planner_period_sec').get_parameter_value().double_value
        self.min_forward_progress_m = self.get_parameter('min_forward_progress_m').get_parameter_value().double_value
        self.forward_score_weight = self.get_parameter('forward_score_weight').get_parameter_value().double_value
        self.goal_republish_distance_m = self.get_parameter('goal_republish_distance_m').get_parameter_value().double_value
        self.startup_forward_distance_m = self.get_parameter('startup_forward_distance_m').get_parameter_value().double_value
        self.startup_forward_lateral_tolerance_m = self.get_parameter('startup_forward_lateral_tolerance_m').get_parameter_value().double_value
        self.general_forward_distance_m = self.get_parameter('general_forward_distance_m').get_parameter_value().double_value
        self.general_forward_lateral_tolerance_m = self.get_parameter('general_forward_lateral_tolerance_m').get_parameter_value().double_value
        self.startup_min_forward_distance_m = self.get_parameter('startup_min_forward_distance_m').get_parameter_value().double_value
        self.startup_goal_hold_sec = self.get_parameter('startup_goal_hold_sec').get_parameter_value().double_value
        self.startup_goal_min_progress_m = self.get_parameter('startup_goal_min_progress_m').get_parameter_value().double_value
        self.goal_reached_distance_m = self.get_parameter('goal_reached_distance_m').get_parameter_value().double_value
        self.goal_replan_timeout_sec = self.get_parameter('goal_replan_timeout_sec').get_parameter_value().double_value
        self.local_planning_radius_m = self.get_parameter('local_planning_radius_m').get_parameter_value().double_value
        self.map_border_margin_cells = self.get_parameter('map_border_margin_cells').get_parameter_value().integer_value

        # Data relating to the robot's current pose and the map data (SLAM)
        self.current_pose = None
        self.current_yaw = 0.0
        self.current_orientation_z = 0.0
        self.current_orientation_w = 1.0
        self.map_data = None

        # Data relating to the robot's current velocity
        self.current_velocity = None
        self.last_goal = None
        self.active_goal_time = None
        self.current_goal_handle = None
        self.pending_goal_future = None
        self.pending_result_future = None
        self.startup_goal_sent = False
        self.startup_goal = None
        self.startup_goal_time = None
        self.startup_pose_x = None
        self.startup_pose_y = None
        self.planning_reference_yaw = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribing to the map topic to get the occupancy grid
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.plan_timer = self.create_timer(planner_period_sec, self.plan_once)

        # Testing parameters
        self.if_comment = True

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

        if self.if_comment:
            self.get_logger().info(
                f"Map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}"
            )


    def navigate_to_waypoint(self, waypoint: PoseStamped):
        if self.last_goal is not None:
            dx = waypoint.pose.position.x - self.last_goal.pose.position.x
            dy = waypoint.pose.position.y - self.last_goal.pose.position.y
            if math.hypot(dx, dy) < self.goal_republish_distance_m:
                return

        self.goal_publisher.publish(waypoint)
        self.last_goal = waypoint
        self.active_goal_time = self.get_clock().now()
        self.get_logger().info(
            f"Published waypoint goal: x={waypoint.pose.position.x:.2f}, y={waypoint.pose.position.y:.2f}"
        )

        if self.pending_goal_future is not None:
            self.get_logger().info("Nav goal request already pending; keeping current goal.")
            return

        if self.current_goal_handle is not None:
            self.get_logger().info("Nav2 goal already active; keeping current goal until it finishes.")
            return

        self._send_nav_goal(waypoint)

    def _send_nav_goal(self, waypoint: PoseStamped):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("Nav2 navigate_to_pose action server is not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.pending_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.pending_goal_future.add_done_callback(self._goal_response_callback)
        self.get_logger().info("Sent waypoint to Nav2 action server.")

    def _goal_response_callback(self, future):
        self.pending_goal_future = None

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send Nav2 goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected waypoint goal.")
            self.current_goal_handle = None
            return

        self.current_goal_handle = goal_handle
        self.pending_result_future = goal_handle.get_result_async()
        self.pending_result_future.add_done_callback(self._goal_result_callback)
        self.get_logger().info("Nav2 accepted waypoint goal.")

    def _goal_result_callback(self, future):
        self.pending_result_future = None
        self.current_goal_handle = None

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get Nav2 goal result: {exc}")
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Waypoint reached successfully.")
        else:
            self.get_logger().warn(f"Waypoint goal finished with status {status}.")

    def update_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(f"Could not get robot pose transform: {exc}")
            return False

        self.current_pose = transform.transform.translation
        q = transform.transform.rotation
        self.current_orientation_z = q.z
        self.current_orientation_w = q.w
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return True

    def get_value_in_map_coords(self, map_x, map_y):
        index = map_y * self.map_data.info.width + map_x
        return self.map_data.data[index]

    def find_best_target(self):
        # This function finds the best target for the robot to navigate to based on the occupancy grid data.
        if self.map_data is None:
            self.get_logger().warn("Map data is not available yet.")
            return None

        if not self.update_robot_pose():
            return None

        if not self.startup_goal_sent:
            self.planning_reference_yaw = self.current_yaw
            startup_target = self.find_startup_forward_target()
            if startup_target is not None:
                return startup_target
        elif self.should_hold_startup_goal():
            return self.startup_goal
        elif self.should_keep_current_goal():
            return self.last_goal

        self.planning_reference_yaw = self.current_yaw

        return self.filter_clusters()

    def map_to_world(self, map_x, map_y):
        info = self.map_data.info
        world_x = info.origin.position.x + (map_x + 0.5) * info.resolution
        world_y = info.origin.position.y + (map_y + 0.5) * info.resolution
        return world_x, world_y

    def world_to_map(self, world_x, world_y):
        info = self.map_data.info
        map_x = int((world_x - info.origin.position.x) / info.resolution)
        map_y = int((world_y - info.origin.position.y) / info.resolution)
        return map_x, map_y

    def _in_bounds(self, x, y, width, height):
        return 0 <= x < width and 0 <= y < height

    def _is_free(self, value):
        return 0 <= value <= self.free_cell_max_value

    def _cell_index(self, x, y, width):
        return y * width + x

    def _euclidean(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def yaw_to_quaternion(self, yaw):
        half_yaw = 0.5 * yaw
        return math.sin(half_yaw), math.cos(half_yaw)

    def is_near_map_border(self, map_x, map_y, width, height):
        margin = self.map_border_margin_cells
        return (
            map_x < margin or
            map_y < margin or
            map_x >= width - margin or
            map_y >= height - margin
        )

    def is_within_local_radius(self, dx_world, dy_world):
        return math.hypot(dx_world, dy_world) <= self.local_planning_radius_m

    def get_planning_forward_vector(self):
        reference_yaw = self.planning_reference_yaw
        if reference_yaw is None:
            reference_yaw = self.current_yaw
        return math.cos(reference_yaw), math.sin(reference_yaw)

    def _make_pose_stamped(self, world_x, world_y):
        target = PoseStamped()
        target.header.frame_id = self.frame_id
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = float(world_x)
        target.pose.position.y = float(world_y)
        target.pose.position.z = 0.0

        # Make this a position-only goal.
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        return target

    def should_hold_startup_goal(self):
        if self.startup_goal is None or self.startup_goal_time is None:
            return False

        elapsed = (self.get_clock().now() - self.startup_goal_time).nanoseconds / 1e9
        if elapsed > self.startup_goal_hold_sec:
            return False

        if self.current_pose is None or self.startup_pose_x is None or self.startup_pose_y is None:
            return True

        moved_distance = math.hypot(
            self.current_pose.x - self.startup_pose_x,
            self.current_pose.y - self.startup_pose_y,
        )
        return moved_distance < self.startup_goal_min_progress_m

    def distance_to_goal(self, goal):
        if goal is None or self.current_pose is None:
            return None

        return math.hypot(
            goal.pose.position.x - self.current_pose.x,
            goal.pose.position.y - self.current_pose.y,
        )

    def should_keep_current_goal(self):
        if self.last_goal is None:
            return False

        if self.current_goal_handle is not None or self.pending_goal_future is not None or self.pending_result_future is not None:
            return True

        distance = self.distance_to_goal(self.last_goal)
        if distance is not None and distance <= self.goal_reached_distance_m:
            return False

        return False

    def find_straight_ahead_target(self, max_distance, min_distance=0.4):
        if self.map_data is None:
            return None

        if not self.update_robot_pose():
            return None

        resolution = self.map_data.info.resolution
        forward_x, forward_y = self.get_planning_forward_vector()

        distance = max_distance
        while distance >= min_distance:
            world_x = self.current_pose.x + distance * forward_x
            world_y = self.current_pose.y + distance * forward_y
            map_x, map_y = self.world_to_map(world_x, world_y)

            if self._in_bounds(map_x, map_y, self.map_data.info.width, self.map_data.info.height):
                if self.is_near_map_border(map_x, map_y, self.map_data.info.width, self.map_data.info.height):
                    distance -= max(resolution, 0.05)
                    continue
                idx = self._cell_index(map_x, map_y, self.map_data.info.width)
                if self._is_free(self.map_data.data[idx]):
                    return self._make_pose_stamped(world_x, world_y), map_x, map_y, distance

            distance -= max(resolution, 0.05)

        return None

    def find_best_forward_cell_in_cluster(self, cluster_cells, forward_x, forward_y):
        best_cell = None
        best_score = -float('inf')
        width = self.map_data.info.width
        height = self.map_data.info.height

        for map_x, map_y in cluster_cells:
            if self.is_near_map_border(map_x, map_y, width, height):
                continue

            world_x, world_y = self.map_to_world(map_x, map_y)
            dx_world = world_x - self.current_pose.x
            dy_world = world_y - self.current_pose.y

            if not self.is_within_local_radius(dx_world, dy_world):
                continue

            forward_progress = dx_world * forward_x + dy_world * forward_y
            lateral_offset = abs(-forward_y * dx_world + forward_x * dy_world)

            if forward_progress < self.min_forward_progress_m:
                continue
            if forward_progress > self.general_forward_distance_m:
                continue
            if lateral_offset > self.general_forward_lateral_tolerance_m:
                continue

            score = 4.0 * forward_progress - 2.0 * lateral_offset
            if score > best_score:
                best_score = score
                best_cell = {
                    'map_x': map_x,
                    'map_y': map_y,
                    'forward_progress': forward_progress,
                    'lateral_offset': lateral_offset,
                    'score': score,
                }

        return best_cell

    def find_forward_target(self, max_forward_distance, lateral_tolerance, min_forward_progress=0.05):
        if self.map_data is None:
            return None

        if not self.update_robot_pose():
            return None

        info = self.map_data.info
        width = info.width
        height = info.height
        data = self.map_data.data

        forward_x, forward_y = self.get_planning_forward_vector()

        best_cell = None
        best_score = -float('inf')

        for map_y in range(height):
            for map_x in range(width):
                if self.is_near_map_border(map_x, map_y, width, height):
                    continue

                idx = self._cell_index(map_x, map_y, width)
                if not self._is_free(data[idx]):
                    continue

                world_x, world_y = self.map_to_world(map_x, map_y)
                dx_world = world_x - self.current_pose.x
                dy_world = world_y - self.current_pose.y

                if not self.is_within_local_radius(dx_world, dy_world):
                    continue

                forward_progress = dx_world * forward_x + dy_world * forward_y
                lateral_offset = abs(-forward_y * dx_world + forward_x * dy_world)

                if forward_progress < min_forward_progress:
                    continue
                if forward_progress > max_forward_distance:
                    continue
                if lateral_offset > lateral_tolerance:
                    continue

                score = 4.0 * forward_progress - 1.5 * lateral_offset
                if score > best_score:
                    best_score = score
                    best_cell = (map_x, map_y, forward_progress)

        if best_cell is None:
            return None

        map_x, map_y, forward_progress = best_cell
        world_x, world_y = self.map_to_world(map_x, map_y)
        return self._make_pose_stamped(world_x, world_y), map_x, map_y, forward_progress

    def find_startup_forward_target(self):
        result = self.find_straight_ahead_target(
            self.startup_forward_distance_m,
            self.startup_min_forward_distance_m
        )
        if result is None:
            result = self.find_forward_target(
                self.startup_forward_distance_m,
                self.startup_forward_lateral_tolerance_m,
                min_forward_progress=max(0.05, self.min_forward_progress_m)
            )
            if result is None:
                self.get_logger().warn("No startup forward target found; falling back to forward cluster search.")
                return None

        target, map_x, map_y, forward_progress = result
        self.get_logger().info(
            f"Startup forward target selected at cell=({map_x}, {map_y}) progress={forward_progress:.2f}"
        )
        return target
        
    def filter_clusters(self):
        if self.map_data is None:
            return None

        info = self.map_data.info
        width = info.width
        height = info.height
        data = self.map_data.data

        visited = set()
        best_cluster = None
        best_score = -1.0
        forward_x, forward_y = self.get_planning_forward_vector()

        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ]

        for y in range(height):
            for x in range(width):
                idx = self._cell_index(x, y, width)
                if (x, y) in visited or not self._is_free(data[idx]):
                    continue

                queue = deque([(x, y)])
                visited.add((x, y))
                cluster_cells = []

                while queue:
                    cx, cy = queue.popleft()
                    cluster_cells.append((cx, cy))

                    for dx, dy in directions:
                        nx, ny = cx + dx, cy + dy
                        if not self._in_bounds(nx, ny, width, height):
                            continue
                        if (nx, ny) in visited:
                            continue

                        nidx = self._cell_index(nx, ny, width)
                        if not self._is_free(data[nidx]):
                            continue

                        visited.add((nx, ny))
                        queue.append((nx, ny))

                if len(cluster_cells) < self.min_cluster_cells:
                    continue

                best_cell = self.find_best_forward_cell_in_cluster(cluster_cells, forward_x, forward_y)
                if best_cell is None:
                    continue

                score = (
                    float(len(cluster_cells))
                    + self.forward_score_weight * best_cell['forward_progress']
                    - 1.5 * best_cell['lateral_offset']
                )

                if score > best_score:
                    best_score = score
                    best_cluster = {
                        'cells': cluster_cells,
                        'size': len(cluster_cells),
                        'forward_progress': best_cell['forward_progress'],
                        'target_cell': (best_cell['map_x'], best_cell['map_y']),
                    }

        if best_cluster is None:
            forward_result = self.find_forward_target(
                self.general_forward_distance_m,
                self.general_forward_lateral_tolerance_m,
                min_forward_progress=max(0.05, self.min_forward_progress_m)
            )
            if forward_result is None:
                self.get_logger().warn("No suitable forward free-space target found.")
                return None

            target, map_x, map_y, forward_progress = forward_result
            self.get_logger().warn(
                f"No forward cluster found. Falling back to forward cell target at ({map_x}, {map_y}) progress={forward_progress:.2f}."
            )
            return target

        target_map_x, target_map_y = best_cluster['target_cell']

        world_x, world_y = self.map_to_world(target_map_x, target_map_y)

        target = self._make_pose_stamped(world_x, world_y)

        self.get_logger().info(
            f"Best cluster size={best_cluster['size']} forward_progress={best_cluster['forward_progress']:.2f} -> target cell=({target_map_x}, {target_map_y})"
        )

        return target

    def plan_once(self):
        waypoint = self.find_best_target()
        if waypoint is None:
            return
        self.navigate_to_waypoint(waypoint)
        if not self.startup_goal_sent:
            self.startup_goal_sent = True
            self.startup_goal = waypoint
            self.startup_goal_time = self.get_clock().now()
            if self.current_pose is not None:
                self.startup_pose_x = self.current_pose.x
                self.startup_pose_y = self.current_pose.y


if __name__ == '__main__':
    

    rclpy.init()
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()