#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs import msg
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Declaring parameters for the waypoint navigator
        self.declare_parameter('map_topic', '/map') # Subscribes to the map topic to get the occupancy grid
        self.declare_parameter('robot_frame', 'base_link') # Subscribes to the robot frame to get the robot's current pose
        self.declare_parameter('frame_id', 'map') # Subscribes to the map frame to get the map's current pose

        # Getting the parameter values
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

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

        self.navigator.setInitialPose(self.initial_pose)

        # self.navigator.waitUntilNav2Active()  # Wait until the navigation stack is active

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

        self.sent_goal = False # Flag to indicate if the goal has been sent to the navigation stack
        self.goal_timer = self.create_timer(1.0, self.send_goal_once) # Timer to send the goal to the navigation stack

        self.waypoints = []

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

        if self.if_comment:
            self.get_logger().info(f"Map data given by the SLAM algorithm: {self.map_data}")


    def navigate_to_waypoint(self, waypoint: PoseStamped):

        self.navigator.goToPose(waypoint)

        while False:

            while not self.navigator.isTaskComplete():
                self.feedback = self.navigator.getFeedback()
                if self.feedback:
                    self.get_logger().info(f"Navigating to waypoint: {self.feedback.current_pose}")

            self.result = self.navigator.getResult()

            if self.result == TaskResult.SUCCEEDED:
                self.get_logger().info("Navigation task succeeded!")
                
            elif self.result == TaskResult.CANCELED:
                self.get_logger().info("Navigation task was canceled.")
                
            elif self.result == TaskResult.FAILED:
                self.get_logger().info("Navigation task failed.")
                
            
            
            

    def get_value_in_map_coords(self, map_x, map_y):
        index = map_y * self.map_data.info.width + map_x
        return self.map_data.data[index]

    def find_best_target(self):
        # This function finds the best target for the robot to navigate to based on the occupancy grid data.
        if self.map_data is None:
            self.get_logger().warn("Map data is not available yet.")
            return None
        
        highest_number = max(self.map_data.data)
        lowest_number = min(self.map_data.data)

    def send_goal_once(self):
        if self.sent_goal:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,          # target frame: map
                self.robot_frame,       # source frame: base_link
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF not ready yet: {ex}")
            return

        self.sent_goal = True

        point = PointStamped()
        point.header.frame_id = self.robot_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = 1.0
        point.point.y = 0.0
        point.point.z = 0.0

        map_point = do_transform_point(point, transform)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = map_point.point.x
        goal_pose.pose.position.y = map_point.point.y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.w = 0.0

        self.get_logger().info(
            f"Sending goal 1m ahead: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
        )

        self.navigate_to_waypoint(goal_pose)
        
    def filter_clusters(self):
        pass

    def create_local_waypoints(self):
        pass


if __name__ == '__main__':
    # import rclpy
    # from rclpy.node import Node

    rclpy.init()
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    rclpy.destroy_node(navigator)
    rclpy.shutdown()
