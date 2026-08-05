#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs import msg
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import Odometry

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Declaring parameters for the waypoint navigator
        self.declare_parameter('map_topic', '/map') # Subscribes to the map topic to get the occupancy grid
        self.declare_parameter('robot_frame', 'base_link') # Subscribes to the robot frame to get the robot's current pose
        self.declare_parameter('frame_id', 'map') # Subscribes to the map frame to get the map's current pose
        self.declare_parameter('odometry_topic', '/odom') # Subscribes to the odometry topic to get the robot's current velocity
        self.declare_parameter('lane_topic', '/lane_markers') # Subscribes to the lane topic to get the lane markers
        
        self.declare_parameter('safety_bubble_radius', 0.5) # Safety bubble radius around the robot to avoid collisions

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Getting the parameter values
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.safety_bubble_radius = self.get_parameter('safety_bubble_radius').get_parameter_value().double_value

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data relating to the robot's current pose and the map data (SLAM)
        self.current_pose = None
        self.map_data = None

        # Data relating to the robot's current velocity
        self.current_velocity = None

        # Subscribing to the map topic to get the occupancy grid
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        # Subscribing to the odometry topic to get the robot's current velocity
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            self.odometry_topic,
            self.odometry_callback,
            10
        )

        self.lane_subscriber = self.create_subscription(
            MarkerArray,
            self.get_parameter('lane_topic').get_parameter_value().string_value,
            self.lane_callback,
            10
        )

        # Testing parameters
        self.if_comment = True

        # Basic Navigator object to handle navigation tasks
        self.navigator = BasicNavigator()

        # Initial parameters / pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = self.frame_id
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.position.z = 0.0
        self.initial_pose.pose.orientation.w = 1.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.navigator.setInitialPose(self.initial_pose)

        # Initial feedback and result for the navigation task
        self.feedback = None # Feedback from the navigation task. Set to None
        self.result = None # Result of the navigation task. Set to None

        # Goal pose for the robot to navigate to
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = self.frame_id
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 1.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        # Most recently calculated forward waypoint
        self.latest_forward_goal = None

        # Last waypoint actually sent to Nav2
        self.last_sent_goal = None

        # Prevent multiple goals being sent simultaneously
        self.goal_in_progress = False

        # Recalculate from map callbacks, but only send at a controlled rate
        self.goal_timer = self.create_timer(
            2.0,
            self.send_latest_forward_goal
        )

        self.initial_goal_send = False
        self.initial_goal_timer = self.create_timer(
            5.0,
            self.send_initial_forward_goal
        )

        self.initial_forward_goal_sent = False

        # Waypoints for the robot to navigate to. This will be a list of PoseStamped objects
        self.local_waypoints = [] # Local waypoints in the robot's frame of reference
        self.global_waypoints = []
        self.raw_grid = [] # Raw occupancy grid data as a 2D numpy array
        self.map_matrix = [] # Occupancy grid data as a 2D numpy array

        # Parameters for the occupancy grid and robot size
        self.inflation_radius = 0.5 # Inflation radius for the occupancy grid to account for the robot's size
        self.cost_scaling_factor = 10.0 # Cost scaling factor for the occupancy grid to account for the robot's size

        # Parameters for lane following
        # Latest lane centreline expressed in map frame
        self.lane_centreline = []

        # Look-ahead distance for lane following
        self.lane_lookahead_distance = 1.5  # metres

        # Prevent old lane detections being used indefinitely
        self.last_lane_update_time = None

        # Parameters for the laser scan
        self.latest_scan = None

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def send_goal(self, goal_pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal_pose.pose.position.x
        goal_msg.pose.pose.position.y = goal_pose.pose.position.y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation = goal_pose.pose.orientation

        if not self.client.server_is_ready():
            self.get_logger().warn(
                "NavigateToPose action server is not ready."
            )
            self.goal_in_progress = False
            return

        self._send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def send_initial_forward_goal(self):
        if self.initial_forward_goal_sent:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,       # target: map
                self.robot_frame,    # source: base_link
                rclpy.time.Time()
            )

        except TransformException as ex:
            self.get_logger().info(
                f"Waiting for map -> base_link TF: {ex}"
            )
            return

        # Current robot pose in map frame
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        robot_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        initial_distance = 1.0  # metres forward

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = (
            robot_x + initial_distance * math.cos(robot_yaw)
        )

        goal.pose.position.y = (
            robot_y + initial_distance * math.sin(robot_yaw)
        )

        goal.pose.position.z = 0.0

        # Keep the robot facing forward
        goal.pose.orientation.z = math.sin(robot_yaw / 2.0)
        goal.pose.orientation.w = math.cos(robot_yaw / 2.0)

        self.get_logger().info(
            f"Sending initial forward goal: "
            f"x={goal.pose.position.x:.2f}, "
            f"y={goal.pose.position.y:.2f}"
        )

        self.initial_forward_goal_sent = True
        self.initial_goal_timer.cancel()

        self.send_goal(goal)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as ex:
            self.get_logger().error(
                f"Failed to send goal: {ex}"
            )
            self.goal_in_progress = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            self.goal_in_progress = False
            return

        self.get_logger().info("Goal accepted!")

        self._goal_handle = goal_handle

        # Allow the next periodically generated waypoint to be sent
        self.goal_in_progress = False

        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            self.arrival_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Current distance or position updates can be read here

    def arrival_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('Robot reached the waypoint!')
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')

    def send_latest_forward_goal(self):
        """
        Send the most recently calculated forward waypoint.

        A new goal is only sent if:
        - a valid forward waypoint exists;
        - no previous goal is currently being submitted;
        - the waypoint has moved far enough from the last sent goal.
        """

        if not self.initial_forward_goal_sent:
            self.get_logger().debug("Initial forward goal not sent yet.")
            return

        if self.latest_forward_goal is None:
            self.get_logger().debug("No forward waypoint available yet.")
            return

        goal_x = self.latest_forward_goal.pose.position.x
        goal_y = self.latest_forward_goal.pose.position.y

        # Do not keep resending almost exactly the same waypoint
        if self.last_sent_goal is not None:
            last_x = self.last_sent_goal.pose.position.x
            last_y = self.last_sent_goal.pose.position.y

            target_change = math.hypot(
                goal_x - last_x,
                goal_y - last_y
            )

            minimum_goal_change = 0.2

            if target_change < minimum_goal_change:
                return

        if self.goal_in_progress:
            return

        self.get_logger().info(
            f"Sending forward waypoint: x={goal_x:.2f}, y={goal_y:.2f}"
        )

        self.goal_in_progress = True
        self.last_sent_goal = self.latest_forward_goal
        
        self.send_goal(self.latest_forward_goal)

    def odometry_callback(self, msg: Odometry):
        self.current_velocity = msg.twist.twist.linear

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert occupancy-grid data into a 2D array.
        self.raw_grid = np.array(msg.data, dtype=np.int8)
        self.map_matrix = self.raw_grid.reshape((height, width))

        try:
            # Get the robot pose in the map frame.
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,       # map
                self.robot_frame,    # base_link
                rclpy.time.Time()
            )

        except TransformException as ex:
            self.get_logger().warn(
                f"TF not ready yet: {ex}",
                throttle_duration_sec=2.0
            )
            return

        # Robot position in the map frame.
        self.robot_x = transform.transform.translation.x
        self.robot_y = transform.transform.translation.y

    

        # Robot orientation in the map frame.
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        self.robot_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        self.current_pose = (self.robot_x, self.robot_y, self.robot_yaw)

        lane_goal = self.calculate_lane_goal()

        if lane_goal is not None:
            self.latest_forward_goal = lane_goal

            self.get_logger().info(
                f"Lane-following goal: "
                f"x={lane_goal.pose.position.x:.2f}, "
                f"y={lane_goal.pose.position.y:.2f}"
            )

            return

        # Convert robot position into occupancy-grid coordinates.
        robot_grid_x = int(
            (self.robot_x - origin_x) / resolution
        )
        robot_grid_y = int(
            (self.robot_y - origin_y) / resolution
        )

        # Make sure the robot itself is inside the map.
        if not (
            0 <= robot_grid_x < width
            and 0 <= robot_grid_y < height
        ):
            self.get_logger().warn(
                "Robot position is outside the occupancy grid."
            )
            return

        # Direction directly in front of the robot.
        forward_vx = math.cos(self.robot_yaw)
        forward_vy = math.sin(self.robot_yaw)

        # Search farther ahead, but only send a nearby goal.
        search_distance = 5.0
        max_goal_distance = 1.5
        minimum_goal_distance = 0.4

        # Permit only a small amount of unknown space.
        #
        # This allows the robot to continue exploring without selecting a goal
        # several metres into completely unobserved space.
        max_unknown_distance = 0.5
        unknown_distance = 0.0

        step_size = resolution
        number_of_steps = int(search_distance / step_size)

        best_grid_x = robot_grid_x
        best_grid_y = robot_grid_y

        found_target = False

        for i in range(1, number_of_steps + 1):
            current_distance = i * step_size

            check_world_x = (
                self.robot_x
                + forward_vx * current_distance
            )
            check_world_y = (
                self.robot_y
                + forward_vy * current_distance
            )

            check_grid_x = int(
                (check_world_x - origin_x) / resolution
            )
            check_grid_y = int(
                (check_world_y - origin_y) / resolution
            )

            # Stop if the ray leaves the occupancy grid.
            if not (
                0 <= check_grid_x < width
                and 0 <= check_grid_y < height
            ):
                break

            cell_value = self.map_matrix[
                check_grid_y,
                check_grid_x
            ]

            if cell_value == 0:
                # Confirmed free space.
                best_grid_x = check_grid_x
                best_grid_y = check_grid_y
                found_target = True

                # Reset because we have returned to known free space.
                unknown_distance = 0.0

            elif cell_value == -1:
                # Unknown space.
                unknown_distance += step_size

                if unknown_distance <= max_unknown_distance:
                    best_grid_x = check_grid_x
                    best_grid_y = check_grid_y
                    found_target = True
                else:
                    break

            else:
                # Positive values represent occupied/probably occupied cells.
                break

        if not found_target:
            self.get_logger().warn(
                "No suitable forward target found."
            )
            return

        # Convert selected grid cell back into map coordinates.
        best_world_x = (
            origin_x
            + (best_grid_x + 0.5) * resolution
        )
        best_world_y = (
            origin_y
            + (best_grid_y + 0.5) * resolution
        )

        dx = best_world_x - self.robot_x
        dy = best_world_y - self.robot_y

        distance_to_target = math.hypot(dx, dy)

        # Limit the actual Nav2 goal to a short look-ahead distance.
        if distance_to_target > max_goal_distance:
            scale = max_goal_distance / distance_to_target

            best_world_x = self.robot_x + dx * scale
            best_world_y = self.robot_y + dy * scale

            distance_to_target = max_goal_distance

        if distance_to_target < minimum_goal_distance:
            self.get_logger().info(
                f"Forward target is too close: "
                f"{distance_to_target:.2f} m"
            )

            # Do not clear latest_forward_goal here.
            # The next map update may produce a valid target.
            return

        forward_goal = PoseStamped()

        forward_goal.header.frame_id = self.frame_id
        forward_goal.header.stamp = (
            self.get_clock().now().to_msg()
        )

        forward_goal.pose.position.x = best_world_x
        forward_goal.pose.position.y = best_world_y
        forward_goal.pose.position.z = 0.0

        # Make the goal face in the current forward direction.
        forward_goal.pose.orientation.x = 0.0
        forward_goal.pose.orientation.y = 0.0
        forward_goal.pose.orientation.z = math.sin(
            self.robot_yaw / 2.0
        )
        forward_goal.pose.orientation.w = math.cos(
            self.robot_yaw / 2.0
        )

        if self.is_waypoint_within_safety_bubble(forward_goal):
            self.get_logger().info(
                "Forward target is within safety bubble."
            )
            return

        # This goal will be sent by send_latest_forward_goal().
        self.latest_forward_goal = forward_goal

        target_cell_value = self.map_matrix[
            best_grid_y,
            best_grid_x
        ]

        self.get_logger().info(
            f"Robot: "
            f"x={self.robot_x:.2f}, "
            f"y={self.robot_y:.2f}, "
            f"yaw={self.robot_yaw:.2f} | "
            f"Forward goal: "
            f"x={best_world_x:.2f}, "
            f"y={best_world_y:.2f}, "
            f"distance={distance_to_target:.2f} m | "
            f"cell={target_cell_value}"
        )
    
    def is_waypoint_within_safety_bubble(self, waypoint: PoseStamped) -> bool:
        """
        Check if a given waypoint is within the safety bubble radius of the robot.

        :param waypoint: The waypoint to check.
        :return: True if the waypoint is within the safety bubble, False otherwise.
        """
        if self.current_pose is None:
            return False

        dx = waypoint.pose.position.x - self.current_pose[0]
        dy = waypoint.pose.position.y - self.current_pose[1]
        distance = math.hypot(dx, dy)

        self.get_logger().warn("Checking waypoint distance: {:.2f} m, safety bubble radius: {:.2f} m".format( distance, self.get_parameter('safety_bubble_radius').get_parameter_value().double_value))
        return distance <= self.get_parameter('safety_bubble_radius').get_parameter_value().double_value

    def lane_callback(self, msg: MarkerArray):
        left_points = []
        right_points = []

        for marker in msg.markers:
            for point in marker.points:
                if marker.ns == "left_lane":
                    left_points.append(
                        (point.x, point.y)
                    )

                elif marker.ns == "right_lane":
                    right_points.append(
                        (point.x, point.y)
                    )

        self.get_logger().info(
            f"Left points: {len(left_points)}, "
            f"Right points: {len(right_points)}"
        )

        if len(left_points) < 2 or len(right_points) < 2:
            self.get_logger().warn(
                "Not enough left/right lane points."
            )
            return

        # Sort each boundary from near to far.
        left_points.sort(
            key=lambda point: math.hypot(
                point[0] - self.robot_x,
                point[1] - self.robot_y
            )
        )

        right_points.sort(
            key=lambda point: math.hypot(
                point[0] - self.robot_x,
                point[1] - self.robot_y
            )
        )

        number_of_pairs = min(
            len(left_points),
            len(right_points)
        )

        centreline = []

        for i in range(number_of_pairs):
            centre_x = (
                left_points[i][0]
                + right_points[i][0]
            ) / 2.0

            centre_y = (
                left_points[i][1]
                + right_points[i][1]
            ) / 2.0

            centreline.append(
                (centre_x, centre_y)
            )

        self.lane_centreline = self.smooth_centerline(
            centreline,
            window_size=3
        )

        self.get_logger().warn(
            f"Updated lane centreline with "
            f"{len(self.lane_centreline)} points"
        )

    def transform_marker_points(
        self,
        points,
        source_frame,
        transform
    ):
        transformed_points = []

        for point in points:
            stamped_point = PointStamped()
            stamped_point.header.frame_id = source_frame
            stamped_point.header.stamp = self.get_clock().now().to_msg()

            stamped_point.point.x = point.x
            stamped_point.point.y = point.y
            stamped_point.point.z = point.z

            transformed = do_transform_point(
                stamped_point,
                transform
            )

            transformed_points.append(
                (
                    transformed.point.x,
                    transformed.point.y
                )
            )

        return transformed_points
    
    def smooth_centerline(self, points, window_size=3):
        if len(points) < window_size:
            return points

        smoothed = []

        for i in range(len(points)):
            start = max(0, i - window_size + 1)
            window = points[start:i + 1]

            average_x = sum(
                point[0] for point in window
            ) / len(window)

            average_y = sum(
                point[1] for point in window
            ) / len(window)

            smoothed.append((average_x, average_y))

        return smoothed

    def calculate_lane_goal(self):
        """
        Select a waypoint that balances:

        - following the lane;
        - moving forwards;
        - moving toward larger free space;
        - maintaining obstacle clearance.

        Unlike the previous version, this searches both forward and lateral
        positions instead of only shifting one fixed forward point.
        """

        if len(self.lane_centreline) < 2:
            return None

        robot_forward_x = math.cos(self.robot_yaw)
        robot_forward_y = math.sin(self.robot_yaw)

        forward_points = []

        for point_x, point_y in self.lane_centreline:
            dx = point_x - self.robot_x
            dy = point_y - self.robot_y

            longitudinal_distance = (
                dx * robot_forward_x
                + dy * robot_forward_y
            )

            if longitudinal_distance > 0.1:
                forward_points.append((point_x, point_y))

        if len(forward_points) < 2:
            return None

        # Sort the centreline from near to far.
        forward_points.sort(
            key=lambda point: math.hypot(
                point[0] - self.robot_x,
                point[1] - self.robot_y
            )
        )

        right_scan_clearance = self.get_scan_clearance(-100, -20)
        left_scan_clearance = self.get_scan_clearance(20, 100)
        front_scan_clearance = self.get_scan_clearance(-20, 20)

        forced_lateral_offset = 0.0

        # Wall on the robot's right: move the waypoint left.
        if right_scan_clearance < 1.8:
            forced_lateral_offset = max(
                0.5,
                min(
                    1.2,
                    1.8 - right_scan_clearance
                )
            )

        # Wall on the robot's left: move the waypoint right.
        elif left_scan_clearance < 1.8:
            forced_lateral_offset = -max(
                0.5,
                min(
                    1.2,
                    1.8 - left_scan_clearance
                )
            )

        # Use shorter candidate distances near obstacles.
        if (
            front_scan_clearance < 0.8
            or right_scan_clearance < 0.7
            or left_scan_clearance < 0.7
        ):
            forward_distances = [0.35, 0.5, 0.7]
        elif (
            front_scan_clearance < 1.4
            or right_scan_clearance < 1.2
            or left_scan_clearance < 1.2
        ):
            forward_distances = [0.5, 0.8, 1.0]
        else:
            forward_distances = [0.8, 1.1, 1.4]

        # Find a centreline point at each requested forward distance.
        centreline_candidates = []

        for target_distance in forward_distances:
            accumulated_distance = 0.0
            selected_index = len(forward_points) - 1

            for i in range(1, len(forward_points)):
                segment_distance = math.hypot(
                    forward_points[i][0] - forward_points[i - 1][0],
                    forward_points[i][1] - forward_points[i - 1][1]
                )

                accumulated_distance += segment_distance

                if accumulated_distance >= target_distance:
                    selected_index = i
                    break

            selected_point = forward_points[selected_index]

            # Robot-relative left direction
            left_x = -math.sin(self.robot_yaw)
            left_y = math.cos(self.robot_yaw)

            right_clearance = self.get_scan_clearance(-100, -20)

            if right_clearance < 1.5:
                left_shift = 0.8

                selected_point = (
                    selected_point[0] + left_shift * left_x,
                    selected_point[1] + left_shift * left_y
                )

                self.get_logger().warn(
                    f"Right wall close ({right_clearance:.2f} m), "
                    f"moving waypoint {left_shift:.2f} m left"
                )

            previous_index = max(0, selected_index - 1)
            next_index = min(
                len(forward_points) - 1,
                selected_index + 1
            )

            tangent_dx = (
                forward_points[next_index][0]
                - forward_points[previous_index][0]
            )

            tangent_dy = (
                forward_points[next_index][1]
                - forward_points[previous_index][1]
            )

            tangent_length = math.hypot(
                tangent_dx,
                tangent_dy
            )

            if tangent_length < 1e-6:
                continue

            tangent_x = tangent_dx / tangent_length
            tangent_y = tangent_dy / tangent_length

            # Make sure the tangent points generally forwards.
            if (
                tangent_x * robot_forward_x
                + tangent_y * robot_forward_y
            ) < 0.0:
                tangent_x = -tangent_x
                tangent_y = -tangent_y

            centreline_candidates.append(
                (
                    selected_point,
                    tangent_x,
                    tangent_y,
                    target_distance
                )
            )

        if not centreline_candidates:
            return None

        # Right wall detected: only permit centre-left and left candidates.
        if right_scan_clearance < 1.5:
            lateral_offsets = [
                0.4,
                0.6,
                0.8,
                1.0,
                1.2
            ]

        # Left wall detected: only permit centre-right and right candidates.
        elif left_scan_clearance < 1.5:
            lateral_offsets = [
                -1.2,
                -1.0,
                -0.8,
                -0.6,
                -0.4
            ]

        # Both sides reasonably clear.
        else:
            lateral_offsets = [
                -0.4,
                -0.2,
                0.0,
                0.2,
                0.4
            ]

        minimum_required_clearance = 1.4

        best_candidate = None
        best_score = -float('inf')

        for (
            selected_point,
            tangent_x,
            tangent_y,
            target_distance
        ) in centreline_candidates:

            # left_normal_x = -tangent_y
            # left_normal_y = tangent_x
            # Use the robot's physical left direction for wall avoidance.
            left_normal_x = -math.sin(self.robot_yaw)
            left_normal_y = math.cos(self.robot_yaw)

            for lateral_offset in lateral_offsets:
                candidate_x = (
                    selected_point[0]
                    + lateral_offset * left_normal_x
                )

                candidate_y = (
                    selected_point[1]
                    + lateral_offset * left_normal_y
                )

                map_clearance = self.get_map_clearance(
                    candidate_x,
                    candidate_y
                )

                # Candidate direction relative to robot.
                candidate_dx = candidate_x - self.robot_x
                candidate_dy = candidate_y - self.robot_y

                candidate_distance = math.hypot(
                    candidate_dx,
                    candidate_dy
                )

                if candidate_distance < 1e-6:
                    continue

                candidate_direction_x = candidate_dx / candidate_distance
                candidate_direction_y = candidate_dy / candidate_distance

                forward_alignment = (
                    candidate_direction_x * robot_forward_x
                    + candidate_direction_y * robot_forward_y
                )

                # Reject goals that are behind the robot.
                if forward_alignment < 0.15:
                    continue

                # Strongly favour free space.
                clearance_score = 15.0 * map_clearance

                # Keep some preference for making progress.
                progress_score = 1.0 * target_distance

                # Prefer alignment with the lane, but not too strongly.
                lane_alignment = (
                    candidate_direction_x * tangent_x
                    + candidate_direction_y * tangent_y
                )

                lane_score = 0.7 * lane_alignment

                # Penalise very large lateral shifts slightly.
                lateral_penalty = 0.25 * abs(lateral_offset)

                # If the wall is on the right, explicitly reward left offsets.
                free_space_bias = 0.0

                if right_scan_clearance < left_scan_clearance:
                    free_space_bias = 2.0 * max(
                        0.0,
                        lateral_offset
                    )

                elif left_scan_clearance < right_scan_clearance:
                    free_space_bias = 2.0 * max(
                        0.0,
                        -lateral_offset
                    )

                score = (
                    clearance_score
                    + progress_score
                    + lane_score
                    + free_space_bias
                    - lateral_penalty
                )

                self.get_logger().info(
                    f"Candidate | "
                    f"forward={target_distance:.2f}, "
                    f"offset={lateral_offset:.2f}, "
                    f"clearance={map_clearance:.2f}, "
                    f"alignment={forward_alignment:.2f}, "
                    f"score={score:.2f}"
                )

                if (
                    map_clearance >= minimum_required_clearance
                    and score > best_score
                ):
                    best_score = score

                    best_candidate = (
                        candidate_x,
                        candidate_y,
                        tangent_x,
                        tangent_y,
                        lateral_offset,
                        target_distance,
                        map_clearance,
                        selected_point
                    )

        if best_candidate is None:
            self.get_logger().warn(
                "No sufficiently clear lane candidate found."
            )
            return None

        (
            adjusted_x,
            adjusted_y,
            tangent_x,
            tangent_y,
            chosen_offset,
            chosen_forward_distance,
            chosen_clearance,
            selected_point
        ) = best_candidate

        # Vector from robot to selected goal, expressed in map frame.
        goal_dx = adjusted_x - self.robot_x
        goal_dy = adjusted_y - self.robot_y

        # Convert goal position into robot-relative coordinates.
        # Positive lateral position means the goal is to the robot's left.
        goal_forward = (
            math.cos(self.robot_yaw) * goal_dx
            + math.sin(self.robot_yaw) * goal_dy
        )

        goal_lateral = (
            -math.sin(self.robot_yaw) * goal_dx
            + math.cos(self.robot_yaw) * goal_dy
        )

        self.get_logger().warn(
            f"Before enforcement | "
            f"forward={goal_forward:.2f}, "
            f"lateral={goal_lateral:.2f}, "
            f"right_clearance={right_scan_clearance:.2f}"
        )

        # If there is a wall on the right, force the final waypoint
        # to be at least this far left of the robot.
        if right_scan_clearance < 1.8:
            minimum_left_distance = 0.9

            if goal_lateral < minimum_left_distance:
                required_left_shift = (
                    minimum_left_distance - goal_lateral
                )

                robot_left_x = -math.sin(self.robot_yaw)
                robot_left_y = math.cos(self.robot_yaw)

                adjusted_x += required_left_shift * robot_left_x
                adjusted_y += required_left_shift * robot_left_y

                goal_lateral = minimum_left_distance

                self.get_logger().error(
                    f"RIGHT WALL: forcing final waypoint "
                    f"{minimum_left_distance:.2f} m to robot's LEFT"
                )

        # Face from the robot toward the selected free-space waypoint.
        goal_yaw = math.atan2(
            adjusted_y - self.robot_y,
            adjusted_x - self.robot_x
        )

        # Hard safety correction:
        # if the right wall is close, ensure the chosen goal is sufficiently left.
        if right_scan_clearance < 1.5 and chosen_offset < 0.6:
            forced_offset = 0.6

            adjusted_x = (
                selected_point[0]
                + forced_offset * left_normal_x
            )

            adjusted_y = (
                selected_point[1]
                + forced_offset * left_normal_y
            )

            chosen_offset = forced_offset

            self.get_logger().warn(
                f"Right wall close at {right_scan_clearance:.2f} m. "
                f"Forcing goal {forced_offset:.2f} m LEFT."
            )

        goal_yaw = math.atan2(
            adjusted_y - self.robot_y,
            adjusted_x - self.robot_x
        )

        # if self.is_waypoint_within_safety_bubble():
        #     self.get_logger().warn(
        #         "Selected waypoint is within safety bubble. "
        #         "Not sending this goal."
        #     )
        #     return None

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = adjusted_x
        goal.pose.position.y = adjusted_y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self.get_logger().warn(
            f"SELECTED FREE-SPACE GOAL | "
            f"forward={chosen_forward_distance:.2f}, "
            f"offset={chosen_offset:.2f}, "
            f"clearance={chosen_clearance:.2f}, "
            f"x={adjusted_x:.2f}, "
            f"y={adjusted_y:.2f}"
        )

        return goal

    def follow_lane(self):
        pass

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_scan_clearance(self, angle_min_deg: float, angle_max_deg: float)->float:

        if self.latest_scan is None:
            self.get_logger().warn("No laser scan data available.")
            return float('inf')

        valid_ranges = []

        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)

        for index, range_value in enumerate(self.latest_scan.ranges):
            angle = self.latest_scan.angle_min + index * self.latest_scan.angle_increment

            if angle_min <= angle <= angle_max:
                if not math.isinf(range_value) and not math.isnan(range_value):
                    valid_ranges.append(range_value)

        if not valid_ranges:
            self.get_logger().warn("No valid scan data in the specified angle range.")
            return float('inf')

        return min(valid_ranges)

    def calculate_avoidance_offset(self) -> float:
        if self.latest_scan is None:
            return 0.0

        front_clearance = self.get_scan_clearance(-20, 20)
        left_clearance = self.get_scan_clearance(20, 100)
        right_clearance = self.get_scan_clearance(-100, -20)

        desired_clearance = 1.5
        maximum_offset = 0.8
        gain = 1.2

        self.get_logger().info(
            f"SCAN: left={left_clearance:.2f}, "
            f"front={front_clearance:.2f}, "
            f"right={right_clearance:.2f}"
        )

        # Positive means move left.
        right_error = max(
            0.0,
            desired_clearance - right_clearance
        )

        # Negative means move right.
        left_error = max(
            0.0,
            desired_clearance - left_clearance
        )

        lateral_offset = gain * (
            right_error - left_error
        )

        lateral_offset = max(
            -maximum_offset,
            min(maximum_offset, lateral_offset)
        )

        self.get_logger().info(
            f"Avoidance offset: {lateral_offset:.2f}"
        )

        return lateral_offset
    
    def get_map_clearance(self, world_x: float, world_y: float) -> float:
        """
        Estimate the distance from a world point to the nearest occupied
        occupancy-grid cell.
        """

        if self.map_data is None or len(self.map_matrix) == 0:
            return 0.0

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        width = self.map_data.info.width
        height = self.map_data.info.height

        centre_grid_x = int((world_x - origin_x) / resolution)
        centre_grid_y = int((world_y - origin_y) / resolution)

        if not (
            0 <= centre_grid_x < width
            and 0 <= centre_grid_y < height
        ):
            return 0.0

        search_radius = 2.0
        search_cells = int(search_radius / resolution)

        minimum_distance = search_radius

        for grid_y in range(
            max(0, centre_grid_y - search_cells),
            min(height, centre_grid_y + search_cells + 1)
        ):
            for grid_x in range(
                max(0, centre_grid_x - search_cells),
                min(width, centre_grid_x + search_cells + 1)
            ):
                cell_value = self.map_matrix[grid_y, grid_x]

                # Treat occupied cells as obstacles.
                if cell_value >= 50:
                    obstacle_world_x = (
                        origin_x + (grid_x + 0.5) * resolution
                    )
                    obstacle_world_y = (
                        origin_y + (grid_y + 0.5) * resolution
                    )

                    distance = math.hypot(
                        obstacle_world_x - world_x,
                        obstacle_world_y - world_y
                    )

                    minimum_distance = min(
                        minimum_distance,
                        distance
                    )

        return minimum_distance
    
    def is_near_obstacle(self, world_x: float, world_y: float, threshold: float = 0.5) -> bool:
        """
        Check if a given world point is near an obstacle in the occupancy grid.

        :param world_x: X coordinate in the map frame.
        :param world_y: Y coordinate in the map frame.
        :param threshold: Distance threshold to consider "near" an obstacle.
        :return: True if near an obstacle, False otherwise.
        """
        clearance = self.get_map_clearance(world_x, world_y)
        return clearance < threshold

if __name__ == '__main__':
    # import rclpy
    # from rclpy.node import Node

    rclpy.init()
    navigator = WaypointNavigator()

    # navigator.send_goal(navigator.goal_pose)  # Send the goal to the navigation stack
    rclpy.spin(navigator)
    rclpy.destroy_node(navigator)
    rclpy.shutdown()
