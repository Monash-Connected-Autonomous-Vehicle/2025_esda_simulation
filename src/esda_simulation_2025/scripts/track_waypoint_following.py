#!/usr/bin/env python3

import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("loop_period", 1.0)
        self.declare_parameter("goal_pause_seconds", 1.0)

        self.declare_parameter("min_frontier_size", 2)
        self.declare_parameter("goal_tolerance", 0.7)
        self.declare_parameter("frontier_search_min_unknown_neighbors", 1)
        self.declare_parameter("progress_timeout_sec", 20.0)
        self.declare_parameter("obstacle_cost_threshold", 40)

        self.declare_parameter("min_goal_distance", 0.2)
        self.declare_parameter("max_goal_distance", 50.0)
        self.declare_parameter("goal_blacklist_radius", 2.0)
        self.declare_parameter("max_recent_goals", 40)
        self.declare_parameter("goal_clearance_radius_cells", 3)

        # Initial startup behavior parameters (not fully implemented yet)
        self.declare_parameter("startup_forward_enabled", True)
        self.declare_parameter("startup_forward_distance", 3.5)

        self.map_topic = str(self.get_parameter("map_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.loop_period = float(self.get_parameter("loop_period").value)
        self.goal_pause_seconds = float(self.get_parameter("goal_pause_seconds").value)
        self.min_frontier_size = int(self.get_parameter("min_frontier_size").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.min_unknown_neighbors = int(
            self.get_parameter("frontier_search_min_unknown_neighbors").value
        )
        self.progress_timeout_sec = float(
            self.get_parameter("progress_timeout_sec").value
        )
        self.obstacle_cost_threshold = int(
            self.get_parameter("obstacle_cost_threshold").value
        )
        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.max_goal_distance = float(self.get_parameter("max_goal_distance").value)
        self.goal_blacklist_radius = float(
            self.get_parameter("goal_blacklist_radius").value
        )
        self.max_recent_goals = int(self.get_parameter("max_recent_goals").value)
        self.goal_clearance_radius_cells = int(
            self.get_parameter("goal_clearance_radius_cells").value
        )

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )

        self.startup_forward_enabled = bool(self.get_parameter("startup_forward_enabled").value)
        self.startup_forward_distance = float(self.get_parameter("startup_forward_distance").value)

        self.startup_phase_active = self.startup_forward_enabled
        self.startup_goal_sent = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_map: Optional[OccupancyGrid] = None
        self.goal_in_progress = False
        self.waiting_after_goal = False
        self.current_goal: Optional[WorldPoint] = None
        self.last_progress_time = self.get_clock().now()

        self.recent_goals: List[WorldPoint] = []
        self.pause_timer = None

        self.timer = self.create_timer(self.loop_period, self.tick)

        self.get_logger().info("Frontier explorer started")
        self.get_logger().info(f"Map topic: {self.map_topic}")
        self.get_logger().info(f"Frame: {self.frame_id}")
        self.get_logger().info(f"Robot base frame: {self.robot_base_frame}")

    def get_robot_pose_and_yaw(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # yaw from quaternion
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return (x, y, yaw)
        except TransformException:
            return None

    def send_startup_forward_goal(self) -> bool:
        pose = self.get_robot_pose_and_yaw()
        if pose is None:
            self.get_logger().warn("Startup phase: could not get robot pose yet")
            return False

        rx, ry, yaw = pose

        # 👇 ADD THIS
        self.get_logger().info(
            f"[STARTUP] Pose: x={rx:.2f}, y={ry:.2f}, "
            f"yaw={yaw:.2f} rad ({math.degrees(yaw):.1f} deg)"
        )


        gx = rx + self.startup_forward_distance * math.cos(yaw)
        gy = ry + self.startup_forward_distance * math.sin(yaw)

        # 👇 ALSO ADD THIS (VERY useful)
        self.get_logger().info(
            f"[STARTUP] Goal: gx={gx:.2f}, gy={gy:.2f}"
        )
        self.send_goal((gx, gy))
        self.startup_goal_sent = True
        return True

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def tick(self):
        if self.latest_map is None:
            self.get_logger().debug("Waiting for map...")
            return

        if self.goal_in_progress or self.waiting_after_goal:
            self.check_progress()
            return

        # Phase 1: move forward once before frontier exploration
        if self.startup_phase_active:
            if not self.startup_goal_sent:
                self.send_startup_forward_goal()
            return

        # Phase 2: frontier exploration
        robot_pose = self.get_robot_position()
        if robot_pose is None:
            self.get_logger().warn("Could not get robot pose from TF yet")
            return

        frontier_goal = self.choose_frontier_goal(robot_pose)
        if frontier_goal is None:
            self.get_logger().info("No usable frontiers found. Exploration may be complete.")
            return

        self.send_goal(frontier_goal)

    def get_robot_position(self) -> Optional[WorldPoint]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except TransformException:
            return None

    def send_goal(self, point: WorldPoint):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("NavigateToPose action server not available yet")
            return

        robot_pose = self.get_robot_position()
        if robot_pose is None:
            self.get_logger().warn("Cannot send goal without robot pose")
            return

        rx, ry = robot_pose
        gx, gy = point
        yaw = math.atan2(gy - ry, gx - rx)
        qz, qw = yaw_to_quaternion(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.goal_in_progress = True
        self.current_goal = point
        self.last_progress_time = self.get_clock().now()
        self.remember_goal(point)

        self.get_logger().info(
            f"Sending frontier goal: ({gx:.2f}, {gy:.2f}), yaw={yaw:.2f}"
        )

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send goal: {e}")
            self.goal_in_progress = False
            self.current_goal = None
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Frontier goal was rejected")
            self.goal_in_progress = False
            self.current_goal = None
            return

        self.get_logger().info("Frontier goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.goal_in_progress = False

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to get result: {e}")
            self.current_goal = None
            return

        status = result.status
        if status == 4:
            self.get_logger().info("Frontier goal succeeded")
        else:
            self.get_logger().warn(f"Frontier goal failed with status {status}")

        self.current_goal = None

        if self.startup_phase_active:
            self.startup_phase_active = False
            self.startup_goal_sent = False
            self.get_logger().info("Startup phase complete, switching to frontier exploration")

        if self.goal_pause_seconds > 0.0:
            self.waiting_after_goal = True
            if self.pause_timer is not None:
                self.pause_timer.cancel()
            self.pause_timer = self.create_timer(
                self.goal_pause_seconds, self.pause_done_once
            )

    def pause_done_once(self):
        self.waiting_after_goal = False
        if self.pause_timer is not None:
            self.pause_timer.cancel()
            self.pause_timer = None

    def check_progress(self):
        if self.current_goal is None:
            return

        robot_pose = self.get_robot_position()
        if robot_pose is None:
            return

        rx, ry = robot_pose
        gx, gy = self.current_goal
        dist = math.hypot(gx - rx, gy - ry)

        if dist < self.goal_tolerance:
            self.last_progress_time = self.get_clock().now()
            return

        elapsed = (
            self.get_clock().now() - self.last_progress_time
        ).nanoseconds / 1e9

        if elapsed > self.progress_timeout_sec:
            self.get_logger().warn(
                "No progress timeout reached; waiting for current Nav2 result."
            )

    def choose_frontier_goal(self, robot_pose: WorldPoint) -> Optional[WorldPoint]:
        if self.latest_map is None:
            return None

        pose = self.get_robot_pose_and_yaw()
        if pose is None:
            self.get_logger().warn("Could not get robot yaw for frontier selection")
            return None

        _, _, robot_yaw = pose

        frontier_clusters = self.find_frontier_clusters(self.latest_map)
        self.get_logger().info(f"Found {len(frontier_clusters)} frontier clusters")

        if not frontier_clusters:
            self.get_logger().info("No frontier clusters found")
            return None

        candidates: List[Tuple[float, WorldPoint, int]] = []

        for i, cluster in enumerate(frontier_clusters):
            self.get_logger().info(f"Cluster {i}: size={len(cluster)}")

            if len(cluster) < self.min_frontier_size:
                self.get_logger().info(
                    f"Cluster {i} rejected: too small (< {self.min_frontier_size})"
                )
                continue

            goal_cell = self.choose_reachable_goal_cell(cluster, robot_pose)
            if goal_cell is None:
                self.get_logger().warn(f"Cluster {i} rejected: no reachable goal cell")
                continue

            gx, gy = goal_cell
            goal_world = self.grid_to_world(self.latest_map, gx, gy)

            dist = math.hypot(
                goal_world[0] - robot_pose[0],
                goal_world[1] - robot_pose[1]
            )

            if dist < self.min_goal_distance:
                self.get_logger().info(
                    f"Cluster {i} rejected: too close ({dist:.2f} < {self.min_goal_distance})"
                )
                continue

            if dist > self.max_goal_distance:
                self.get_logger().info(
                    f"Cluster {i} rejected: too far ({dist:.2f} > {self.max_goal_distance})"
                )
                continue

            goal_yaw = math.atan2(
                goal_world[1] - robot_pose[1],
                goal_world[0] - robot_pose[0]
            )

            yaw_error = math.atan2(
                math.sin(goal_yaw - robot_yaw),
                math.cos(goal_yaw - robot_yaw)
            )

            forward_score = math.cos(yaw_error)

            if forward_score < -0.2:
                self.get_logger().info(
                    f"Cluster {i} rejected: too far off forward direction "
                    f"(forward_score={forward_score:.2f})"
                )
                continue

            # Only allow frontier goals roughly in front of the robot
            # 45 deg each side = 90 deg forward cone
            if abs(yaw_error) > math.radians(45.0):
                self.get_logger().info(
                    f"Cluster {i} rejected: outside forward sector "
                    f"(yaw_error={math.degrees(yaw_error):.1f} deg)"
                )
                continue

            if self.is_blacklisted_goal(goal_world):
                self.get_logger().info(f"Cluster {i} rejected: blacklisted")
                continue

            score = self.score_frontier_cluster(cluster, goal_world, robot_pose)

            self.get_logger().info(
                f"Cluster {i} accepted: goal=({goal_world[0]:.2f}, {goal_world[1]:.2f}), "
                f"dist={dist:.2f}, yaw_error_deg={math.degrees(yaw_error):.1f}, "
                f"score={score:.2f}"
            )

            candidates.append((score, goal_world, len(cluster)))

        if not candidates:
            self.get_logger().error("No candidates survived filtering")
            return None

        candidates.sort(key=lambda x: x[0], reverse=True)
        best_score, best_goal, best_size = candidates[0]

        self.get_logger().info(
            f"Selected frontier goal ({best_goal[0]:.2f}, {best_goal[1]:.2f}), "
            f"size={best_size}, score={best_score:.2f}"
        )
        return best_goal

    def score_frontier_cluster(
        self,
        cluster,
        goal_world,
        robot_pose,
    ) -> float:
        dist = math.hypot(
            goal_world[0] - robot_pose[0],
            goal_world[1] - robot_pose[1]
        )

        pose = self.get_robot_pose_and_yaw()
        heading_penalty = 0.0
        forward_bonus = 0.0

        if pose is not None:
            _, _, robot_yaw = pose

            goal_yaw = math.atan2(
                goal_world[1] - robot_pose[1],
                goal_world[0] - robot_pose[0]
            )

            yaw_error = math.atan2(
                math.sin(goal_yaw - robot_yaw),
                math.cos(goal_yaw - robot_yaw)
            )

            abs_err = abs(yaw_error)

            heading_penalty = abs_err * 6.0
            forward_bonus = 3.0 * math.cos(abs_err)

        return (2.0 * len(cluster)) - dist - heading_penalty + forward_bonus

    def world_to_grid(self, map_msg: OccupancyGrid, wx: float, wy: float) -> Optional[GridCell]:
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        gx = int((wx - origin_x) / resolution)
        gy = int((wy - origin_y) / resolution)

        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return None

        return (gx, gy)
    
    def evaluate_goal_cell(
        self, map_msg: OccupancyGrid, x: int, y: int
    ) -> Tuple[bool, float, float]:
        r = self.goal_clearance_radius_cells

        unknown_count = 0
        total_count = 0
        nearest_obstacle_dist = float("inf")

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx = x + dx
                ny = y + dy

                val = self.get_cell(map_msg, nx, ny)
                if val is None:
                    return (False, 0.0, 1.0)

                total_count += 1

                if val == -1:
                    unknown_count += 1
                    continue

                # Anything at or above threshold is considered too close to obstacle
                if val >= self.obstacle_cost_threshold:
                    return (False, 0.0, 1.0)

                # Track nearest non-free cost as a proxy for obstacle proximity
                if val > 0:
                    dist = math.hypot(dx, dy)
                    if dist < nearest_obstacle_dist:
                        nearest_obstacle_dist = dist

        unknown_ratio = unknown_count / total_count if total_count > 0 else 1.0

        # Make this stricter than your current 0.6
        if unknown_ratio > 0.20:
            return (False, 0.0, unknown_ratio)

        # If no nonzero-cost cells nearby, treat as very open
        if nearest_obstacle_dist == float("inf"):
            clearance_score = float(r + 1)
        else:
            clearance_score = nearest_obstacle_dist

        return (True, clearance_score, unknown_ratio)

    def choose_reachable_goal_cell(
        self, cluster: List[GridCell], robot_pose: WorldPoint
    ) -> Optional[GridCell]:
        if self.latest_map is None or not cluster:
            self.get_logger().warn("choose_reachable_goal_cell: missing map or empty cluster")
            return None

        avg_x = sum(c[0] for c in cluster) / len(cluster)
        avg_y = sum(c[1] for c in cluster) / len(cluster)

        robot_cell = self.world_to_grid(self.latest_map, robot_pose[0], robot_pose[1])
        if robot_cell is None:
            self.get_logger().warn("choose_reachable_goal_cell: robot pose not in map")
            return None

        rx, ry = robot_cell
        dx = avg_x - rx
        dy = avg_y - ry
        mag = math.hypot(dx, dy)

        if mag < 1e-6:
            self.get_logger().warn("choose_reachable_goal_cell: centroid too close to robot")
            return None

        ux = dx / mag
        uy = dy / mag

        # Search farther back from the frontier centroid
        max_backoff = 80

        best_cell = None
        best_score = -float("inf")

        checked_free = 0
        checked_safe = 0

        for step in range(max_backoff + 1):
            gx = int(round(avg_x - ux * step))
            gy = int(round(avg_y - uy * step))

            val = self.get_cell(self.latest_map, gx, gy)
            if val is None:
                continue

            if val != 0:
                continue

            checked_free += 1

            safe, clearance_score, unknown_ratio = self.evaluate_goal_cell(
                self.latest_map, gx, gy
            )

            if not safe:
                continue

            checked_safe += 1

            # Prefer cells with:
            # 1) larger clearance from obstacles
            # 2) less nearby unknown
            # 3) some backoff from the frontier edge
            score = (
                3.0 * clearance_score
                - 2.0 * unknown_ratio
                + 0.05 * step
            )

            if score > best_score:
                best_score = score
                best_cell = (gx, gy)

        if best_cell is not None:
            self.get_logger().info(
                f"choose_reachable_goal_cell: selected best safe cell={best_cell}, "
                f"score={best_score:.3f}, free_checked={checked_free}, safe_checked={checked_safe}"
            )
            return best_cell

        self.get_logger().warn(
            f"choose_reachable_goal_cell failed: free_cells_checked={checked_free}, "
            f"safe_cells_checked={checked_safe}"
        )
        return None

    def find_frontier_clusters(self, map_msg: OccupancyGrid) -> List[List[GridCell]]:
        width = map_msg.info.width
        height = map_msg.info.height

        frontier_cells: List[GridCell] = []
        frontier_set = set()

        for y in range(height):
            for x in range(width):
                if self.is_frontier_cell(map_msg, x, y):
                    frontier_cells.append((x, y))
                    frontier_set.add((x, y))

        visited = set()
        clusters: List[List[GridCell]] = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster: List[GridCell] = []
            queue = deque([cell])
            visited.add(cell)

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))

                for nx, ny in self.neighbors8(cx, cy):
                    if (nx, ny) in frontier_set and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))

            clusters.append(cluster)

        return clusters

    def is_frontier_cell(self, map_msg: OccupancyGrid, x: int, y: int) -> bool:
        value = self.get_cell(map_msg, x, y)
        if value != 0:
            return False

        unknown_neighbors = 0
        for nx, ny in self.neighbors8(x, y):
            nval = self.get_cell(map_msg, nx, ny)
            if nval is None:
                continue
            if nval == -1:
                unknown_neighbors += 1

        return unknown_neighbors >= self.min_unknown_neighbors

    def is_goal_cell_safe(self, map_msg: OccupancyGrid, x: int, y: int) -> bool:
        r = self.goal_clearance_radius_cells

        unknown_count = 0
        total_count = 0

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                val = self.get_cell(map_msg, x + dx, y + dy)
                if val is None:
                    return False

                total_count += 1

                if val >= self.obstacle_cost_threshold:
                    return False

                if val == -1:
                    unknown_count += 1

        return unknown_count <= total_count * 0.6

    def is_blacklisted_goal(self, goal: WorldPoint) -> bool:
        for old_goal in self.recent_goals:
            if math.hypot(goal[0] - old_goal[0], goal[1] - old_goal[1]) < self.goal_blacklist_radius:
                return True
        return False

    def remember_goal(self, goal: WorldPoint):
        self.recent_goals.append(goal)
        if len(self.recent_goals) > self.max_recent_goals:
            self.recent_goals.pop(0)

    def get_cell(self, map_msg: OccupancyGrid, x: int, y: int) -> Optional[int]:
        width = map_msg.info.width
        height = map_msg.info.height

        if x < 0 or y < 0 or x >= width or y >= height:
            return None

        idx = y * width + x
        return map_msg.data[idx]

    def grid_to_world(self, map_msg: OccupancyGrid, gx: int, gy: int) -> WorldPoint:
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution
        return (wx, wy)

    def neighbors8(self, x: int, y: int) -> List[GridCell]:
        return [
            (x - 1, y - 1), (x, y - 1), (x + 1, y - 1),
            (x - 1, y),                 (x + 1, y),
            (x - 1, y + 1), (x, y + 1), (x + 1, y + 1),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()