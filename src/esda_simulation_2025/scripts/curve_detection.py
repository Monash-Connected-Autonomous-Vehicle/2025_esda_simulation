#!/usr/bin/env python3

"""
This is a ROS2 Node that returns if whether a curve has been detected in front of the robot

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, Point
from nav_msgs.msg import OccupancyGrid
import numpy as np

from visualization_msgs.msg import Marker


from tf2_ros import Buffer, TransformListener, TransformException

class CurveDetectionNode(Node):
    def __init__(self):
        super().__init__('curve_detection')

        # 1. Lookahead distance parameters
        self.declare_parameter('curve_lookahead_distant', 3.0) # Lookahead distance in metres
        self.declare_parameter('curve_width', 2.0)

        # Getting the distance parameters
        self.curve_lookahead_distant = (
            self.get_parameter('curve_lookahead_distant')
            .get_parameter_value()
            .double_value
        )

        self.curve_width = (
            self.get_parameter('curve_width')
            .get_parameter_value()
            .double_value
        )

        # TF parameters
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the Local Costmap for curve detection and avoidance
        self.occupancy_grid_subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.occupancy_listener_callback,
            10
        )

        # Publishes the band points to RVIZ for visualization of the centre band points
        self.marker_publisher = self.create_publisher(
            Marker,
            '/curve_detection/centers',
            10
        )

        self.get_logger().info("Curve Detection Node started")

    def occupancy_listener_callback(self, msg):
        # Callback function that executes whenever a scan of the Occupancy Grid occurs
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Tells us which costmap to use
        costmap_frame = msg.header.frame_id

        grid = np.array(msg.data).reshape((height, width))

        occupied_cells = np.argwhere(grid >= 50)

        try:
            transform = self.tf_buffer.lookup_transform('base_link', costmap_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform "
                f"{costmap_frame} -> base_link: {ex}"
            )

            return

        obstacles_in_front = []

        for row, col in occupied_cells:
            x_costmap = origin_x + (col + 0.5) * resolution
            y_costmap = origin_y + (row + 0.5) * resolution


            # Create point in costmap frame
            point = PointStamped()
            point.header.frame_id = costmap_frame

            point.point.x = x_costmap
            point.point.y = y_costmap
            point.point.z = 0.0

            # Apply the point transformation
            point_base = self.transform_point(point, transform)

            x_base = point_base[0]
            y_base = point_base[1]

            if (0.0 < x_base < self.curve_lookahead_distant and abs(y_base) < self.curve_width):
                obstacles_in_front.append((x_base, y_base))

        # Finding the centre for each distance band
        near = self.get_band_centre(obstacles_in_front, 0.5, 1.0)

        mid = self.get_band_centre(obstacles_in_front, 1.0, 2.0)

        far = self.get_band_centre(obstacles_in_front, 2.0, 3.0)

        self.publish_centres(near, mid, far)

        self.get_logger().info(
            f"Obstacles in front: "
            f"{len(obstacles_in_front)}"
        )

        self.get_logger().info(
            f"Near={near} | Mid={mid} | Far={far}"
        )

        # TEMPORARY DEBUGGING
        # for x, y in obstacles_in_front:

        #     self.get_logger().info(
        #         f"Obstacle: "
        #         f"{x:.2f} m forward, "
        #         f"{y:.2f} m lateral"
        #     )

    def transform_point(self, point, transform):
        """
        Apply a 2D TF transform manually.

        Returns the point in base_link coordinates.
        """

        # Translation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        
        # Quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        # Quaternion --> Yaw
        yaw = np.arctan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        x = point.point.x
        y = point.point.y

        # Rotate + translate
        x_transformed = (
            cos_yaw * x
            - sin_yaw * y
            + tx
        )

        y_transformed = (
            sin_yaw * x
            + cos_yaw * y
            + ty
        )

        return x_transformed, y_transformed

    def get_band_centre(self, obstacles, x_min, x_max):
        """
        Defines the centre of the track. This is intended to detect for curvature on the track
        """

        band_points = [(x, y) for x, y in obstacles if x_min <= x <= x_max]

        if len(band_points) < 2 : 
            return

        ys = [p[1] for p in band_points]

        left_boundary = max(ys)
        right_boundary = min(ys)

        centre_y = (left_boundary + right_boundary) / 2.0

        centre_x = (x_min + x_max) / 2.0

        return (centre_x, centre_y)

    def publish_centres(self, near, mid, far):
        """
        Publishes the markers 'near', 'mid' and 'far' into RVIZ
        """
        
        # Creating a marker instance
        point_to_publish = Marker()

        # Defining the characteristics of the point_to_publish
        point_to_publish.header.frame_id = 'base_link'
        point_to_publish.header.stamp.sec = 0
        point_to_publish.header.stamp.nanosec = 0

        point_to_publish.ns = 'curve_centres'
        point_to_publish.id = 0

        point_to_publish.type = Marker.SPHERE_LIST
        point_to_publish.action = Marker.ADD

        # Size of each point
        point_to_publish.scale.x = 0.20
        point_to_publish.scale.y = 0.20
        point_to_publish.scale.z = 0.20

        # Marker colour 
        point_to_publish.color.r = 1.0
        point_to_publish.color.g = 0.0
        point_to_publish.color.b = 0.0
        point_to_publish.color.a = 1.0

        # Marker position
        for centre in [near, mid, far]:
            if centre is None:
                continue
            
            point = Point()
            point.x = centre[0]
            point.y = centre[1]
            point.z = 0.1

            point_to_publish.points.append(point)

        self.marker_publisher.publish(point_to_publish)


if __name__ == '__main__':
    rclpy.init()
    curve_detector = CurveDetectionNode()

    # navigator.send_goal(navigator.goal_pose)  # Send the goal to the navigation stack
    rclpy.spin(curve_detector)
    rclpy.destroy_node(curve_detector)
    rclpy.shutdown()
