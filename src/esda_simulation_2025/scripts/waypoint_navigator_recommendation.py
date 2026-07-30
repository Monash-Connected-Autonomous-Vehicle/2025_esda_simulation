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

            minimum_goal_change = 0.5

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

        dx = waypoint.pose.position.x - self.current_pose.pose.position.x
        dy = waypoint.pose.position.y - self.current_pose.pose.position.y
        distance = math.hypot(dx, dy)

        return distance <= self.get_parameter('safety_bubble_radius').get_parameter_value().double_value

    def lane_callback(self, msg: MarkerArray):
        # Process lane markers from the MarkerArray message
        # For now, just log the number of markers received
        # self.get_logger().info(f"Received {len(msg.markers)} lane markers.")
        pass

    def follow_lane(self):
        pass

if __name__ == '__main__':
    # import rclpy
    # from rclpy.node import Node

    rclpy.init()
    navigator = WaypointNavigator()

    # navigator.send_goal(navigator.goal_pose)  # Send the goal to the navigation stack
    rclpy.spin(navigator)
    rclpy.destroy_node(navigator)
    rclpy.shutdown()
