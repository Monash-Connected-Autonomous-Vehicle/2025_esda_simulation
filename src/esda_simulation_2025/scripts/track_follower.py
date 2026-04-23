#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import subprocess
import math

import rclpy

from rclpy.node import Node

# from build.esda_simulation_2025.rosidl_generator_py.esda_simulation_2025.msg._navigation_recommendation import NavigationRecommendation
from esda_simulation_2025.msg import NavigationRecommendation


# This node is the main node that will handle the overall track following around the entire map. It will subscribe to the laser scan data and implement the follow the gap algorithm to navigate around the track. It will also handle switching to a different navigation strategy when the robot is within a certain distance to the goal.
# The pipeline will be as follows:
# 1. Get the track direction --> Uses lane detection, obstacle detection and corridors from LiDAR data to determine the direction of the track and the location of the goal.
# 2. Follow the gap algorithm to navigate around the track until the robot is within a certain distance to the goal.
# 3. Converts to motion e.g. Angular Velocity, Linear Velocity to navigate towards the goal.
# 4. Publish /cmd_vel to move the robot.

class TrackFollower(Node):
    def __init__(self):
        super().__init__('track_follower')

        # Declaring various topics and frames
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('test_track_follower_itself', True)  # Parameter to control whether to test the track follower node itself or to use the behaviour tree to manage it. This will allow us to test the track follower node in isolation without needing to run the entire behaviour tree, which can be useful for debugging and development purposes.

        # Declaring to disable to enable cmd_vel for testing
        self.declare_parameter('enable_cmd_vel', True)  # Parameter to control whether to enable publishing cmd_vel from this node. This can be useful for testing and development purposes, allowing us to disable cmd_vel publishing when we want to test the behaviour tree's decision making without actually moving the robot.
        self.enable_cmd_vel = self.get_parameter('enable_cmd_vel').get_parameter_value().bool_value

        # Declaring gains
        self.declare_parameter('K_steering', 1.0)  # Gain for the steering control, this can be adjusted based on the desired responsiveness of the steering control. A higher gain will result in more aggressive steering, while a lower gain will result in smoother but less responsive steering.
        self.declare_parameter('K_speed', 0.5)  # Gain for the speed control, this can be adjusted based on the desired responsiveness of the speed control. A higher gain will result in more aggressive speed changes, while a lower gain will result in smoother but less responsive speed control.
        self.K_steering = self.get_parameter('K_steering').get_parameter_value().double_value
        self.K_speed = self.get_parameter('K_speed').get_parameter_value().double_value


        self.test_track_follower_itself = self.get_parameter('test_track_follower_itself').get_parameter_value().bool_value

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) 
        self.goal_distance_threshold = 0.5  # Distance threshold to switch to goal navigation
        self.current_goal = None  # Placeholder for the current goal position

        
        self.track_width = 4.0 # Assume a 4 metre track width for computing the centreline from the lane detection data, this can be adjusted based on the actual track width in the simulation or real world environment.
        self.one_lane_threshold_factor = 1.2  # Threshold for determining if we only see one lane, this can be adjusted based on the expected distance between the lanes and the noise in the lane detection data. If the average x position of the detected lane points is within this threshold from the desired offset, we can consider that we only see one lane and adjust our control strategy accordingly.

        self.left_lane = []
        self.right_lane = []

        # Subscribes to LiDAR laser scan data
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            self.get_parameter('lidar_topic').get_parameter_value().string_value, 
            self.lidar_listener_callback, 
            10
        )

        # Subscribes to lane detection data (if available) to assist in determining track direction
        self.lane_subscriber = self.create_subscription(
            MarkerArray, 
            '/lane_markers', 
            self.lane_listener_callback, 
            10
        )

        # self.behaviour_tree_subscriber = self.create_subscription(
        #     Twist, 
        #     '/cmd_vel', 
        #     self.track_follower_callback, 
        #     10
        # )

        self.behaviour_tree_publisher = self.create_publisher(
            NavigationRecommendation,
            '/track_follower_recommendation', 
            10
        )

        # Creates a timer to periodically compute the centreline and desired heading based on the latest LiDAR and lane detection data
        self.timer = self.create_timer(0.1, self.lane_timer_callback)

        print("Track Follower node initialized and subscribed to LiDAR and lane detection topics.")


    def lane_timer_callback(self):
        # This timer callback will be called periodically to compute the centreline and desired heading based on the latest LiDAR and lane detection data. It will then publish the appropriate cmd_vel to navigate towards the goal.
        
        # No centre line if we don't have both left and right lane points, so we can't compute a desired heading or navigate towards the goal without a centreline to follow. We will need to wait until we have both left and right lane points before we can compute the centreline and desired heading.
        if not self.left_lane or not self.right_lane:
            return
        
        centreline = self.compute_centreline()

        if not centreline:
            return
        
        desired_offset = self.track_width / 2.0  # Desired offset from the centreline to follow, this can be adjusted based on the desired position of the robot on the track (e.g. closer to the left or right lane). For now, we will aim to follow the centreline, so the desired offset is half the track width.
        print(f"self.left_lane: {self.left_lane}")
        print(f"self.right_lane: {self.right_lane}")
        # Check if we only see one of the lanes
        if len(self.left_lane) > len(self.right_lane) * self.one_lane_threshold_factor:
            print("Only left lane detected, cannot compute centreline.")
            avg_x = np.mean([point[0] for point in self.left_lane])

            error = desired_offset - abs(avg_x)  # Compute the error based on the average x position of the detected lane points and the desired offset from the centreline. This is a simple proportional control approach where we compute the error as the difference between the desired offset and the actual offset based on the detected lane points, and we can use this error to compute a steering command to try to maintain the desired offset from the lane. We can adjust this to include a more sophisticated control approach if needed, such as a PID controller or a pure pursuit controller.

            steering = self.K_steering * error  # Compute the steering command based on the error and the steering gain. This will determine how aggressively the robot tries to correct its position based on the detected lane points.
            print(f"{steering=:.2f}, {error=:.2f}, {avg_x=:.2f}")
            return
        elif len(self.right_lane) > len(self.left_lane) * self.one_lane_threshold_factor:
            print("Only right lane detected, cannot compute centreline.")
            avg_x = np.mean([point[0] for point in self.right_lane])
            error = desired_offset - abs(avg_x)  # Compute the error based on the average x position of the detected lane points and the desired offset from the centreline.

            steering = self.K_steering * error  # Compute the steering command based on the error and the steering gain.
            print(f"{steering=:.2f}, {error=:.2f}, {avg_x=:.2f}")
            return
        
        # For debugging purposes, we will print out the computed centreline points and the number of points in the centreline. This will help us verify that we are correctly computing the centreline from the lane detection data. We will also check that we are correctly matching points from the left and right lanes to compute the centreline points, and that we are only considering matches that are within a certain distance along the track to ensure we are matching points that are close enough together.
        idx = min(8, len(centreline) - 1)
        target_x, target_y, target_z = centreline[idx]

        # Experiemtn with this or target_x
        steering_error = math.atan2(target_x, target_z)  # Compute the steering error based on the target point in the centreline. This is a simple proportional control approach where we compute the angle to the target point and use that as the steering command. We can adjust this to include a more sophisticated control approach if needed, such as a PID controller or a pure pursuit controller.

        if self.test_track_follower_itself:
            cmd = Twist()
            cmd.linear.x = 0.5  # Set a constant forward speed, this can be adjusted based on the distance to the target or other factors
            cmd.angular.z = -1.0 * steering_error  # Proportional control for steering based on the error to the target
            

            self.cmd_vel_pub.publish(cmd)

        else:
            # Sends behaviour_tree the computed centreline and desired heading to assist in determining the track direction and goal location, and to assist in switching between different navigation strategies based on the distance to the goal. The behaviour tree will then use this information to determine which navigation strategy to use (e.g. follow the gap, goal navigation, etc.) and to compute the appropriate cmd_vel to publish to navigate towards the goal.
            pass

        print(f"The centreline points are: {centreline}")
        print(f"Computed centreline with {len(centreline)} points.")
        # Here we would also compute the desired heading based on the centreline and LiDAR data, and then publish the cmd_vel to navigate towards the goal. This is where we would implement the logic to switch to a different navigation strategy when we are within a certain distance to the goal.

    def lidar_listener_callback(self, msg):
        # Access LiDAR data from the LaserScan message
        ranges = msg.ranges

    def lane_listener_callback(self, msg):
        # Access lane detection data from the MarkerArray message
        markers = msg.markers

        left_points = []
        right_points = []

        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z

            point = (x, y, z)

            if marker.ns == 'left_lane':
                left_points.append(point)
            elif marker.ns == 'right_lane':
                right_points.append(point)
        
        self.left_lane = left_points
        self.right_lane = right_points

        print(f"len(self.left_lane): {len(self.left_lane)}")
        print(f"len(self.right_lane): {len(self.right_lane)}")

    def compute_centreline(self):
        # Can't compute a centreline if we don't have both left and right lane points
        if not self.left_lane or not self.right_lane:
            return []

        # Sort the lane points by their z-coordinate (distance along the track) to ensure we are matching points that are at similar positions along the track
        left_sorted = sorted(self.left_lane, key=lambda p: p[2])
        right_sorted = sorted(self.right_lane, key=lambda p: p[2])

        # Determine which lane has fewer points to use as the primary for matching, to ensure we are not trying to match more points than we have in the other lane
        if len(left_sorted) <= len(right_sorted):
            primary = left_sorted
            secondary = right_sorted
            primary_is_left = True
        else:
            primary = right_sorted
            secondary = left_sorted
            primary_is_left = False

        # Match points from the primary lane to the closest point in the secondary lane based on their z-coordinate (distance along the track) to compute the centreline points. We will only consider matches that are within a certain z-distance threshold to ensure we are matching points that are close enough along the track.
        centreline = []
        max_z_diff = 0.5

        # For each point in the primary lane, find the closest point in the secondary lane based on their z-coordinate (distance along the track) and compute the centreline point as the midpoint between the two matched points. We will only consider matches that are within a certain z-distance threshold to ensure we are matching points that are close enough along the track.
        for px, py, pz in primary:
            best_match = None
            best_diff = float('inf')

            # Iterate through the secondary lane points to find the closest point based on z-coordinate (distance along the track)
            for sx, sy, sz in secondary:
                diff = abs(sz - pz)
                if diff < best_diff:
                    best_diff = diff
                    best_match = (sx, sy, sz)

            # Only consider matches that are within the z-distance threshold to ensure we are matching points that are close enough along the track
            if best_match is None or best_diff > max_z_diff:
                continue

            sx, sy, sz = best_match

            if primary_is_left:
                lx, ly, lz = px, py, pz
                rx, ry, rz = sx, sy, sz
            else:
                lx, ly, lz = sx, sy, sz
                rx, ry, rz = px, py, pz

            centreline.append((
                (lx + rx) / 2.0,
                (ly + ry) / 2.0,
                (lz + rz) / 2.0
            ))

        return centreline

    def compute_desired_heading(self, scan_data):
        # Implement logic to compute the desired heading based on LiDAR data and lane detection
        # This could involve analyzing the gaps in the LiDAR data and the lane markers to determine the best direction to follow

        pass

    def get_track_direction(self, scan_data):
        # Implement logic to determine the track direction based on LiDAR data

        pass




    def navigate_to_goal(self):
        # Implement navigation logic to move towards the current goal
        pass

    def get_line_equation(self):
        # Implement logic to compute the line equation for the track direction based on lane detection and LiDAR data. This can be used to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal.

        pass

    


def main(args=None):
    rclpy.init(args=args)
    track_follower = TrackFollower()
    rclpy.spin(track_follower)
    track_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()