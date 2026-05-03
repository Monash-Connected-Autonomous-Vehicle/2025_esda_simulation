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
from esda_simulation_2025.msg import NavigationRecommendation, LaneParameters


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

        self.declare_parameter('test_track_follower_itself', False)  # Parameter to control whether to test the track follower node itself or to use the behaviour tree to manage it. This will allow us to test the track follower node in isolation without needing to run the entire behaviour tree, which can be useful for debugging and development purposes.

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

        # Publisher for lane parameters to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal. This will allow us to publish the computed lane parameters (e.g. lane curvature, lane offset, etc.) to a topic that can be subscribed to by the behaviour tree or other nodes to assist in decision making for navigation.
        self.lane_parameters_publisher = self.create_publisher(LaneParameters, '/lane_parameters', 10)  

        self.goal_distance_threshold = 0.5  # Distance threshold to switch to goal navigation
        self.current_goal = None  # Placeholder for the current goal position

        
        self.track_width = 4.0 # Assume a 4 metre track width for computing the centreline from the lane detection data, this can be adjusted based on the actual track width in the simulation or real world environment.
        self.one_lane_threshold_factor = 1.2  # Threshold for determining if we only see one lane, this can be adjusted based on the expected distance between the lanes and the noise in the lane detection data. If the average x position of the detected lane points is within this threshold from the desired offset, we can consider that we only see one lane and adjust our control strategy accordingly.

        self.temp_lane_timer_cb_test = True  # Temporary variable to control whether to run the lane_timer_callback for testing purposes, this can be removed once we have the lane_timer_callback fully implemented and tested.

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

    def get_line_lane_equation(self, lane_points):
        # Implement logic to compute the line equation for a lane based on the detected lane points. This can be used to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        # Getting the line equation for the left and right lanes can help us determine the track direction and can also assist in switching between different navigation strategies based on the distance to the goal. For example, if we are far from the goal, we may want to follow the lane more closely, while if we are close to the goal, we may want to switch to a different navigation strategy that focuses more on navigating towards the goal rather than following the lane.
        if lane_points is None or len(lane_points) < 2:
            return None  # Not enough points to compute a line equation
        
        # Extract x and z coordinates from the lane points
        z_values = np.array([point[2] for point in lane_points], dtype=float)
        x_values = np.array([point[0] for point in lane_points], dtype=float)

        # Fit a line of the form x = m*z + c to the lane points using numpy's polyfit function, treating z as the independent variable and x as the dependent variable. This will give us the slope (m) and intercept (c) of the line equation, which we can use to determine the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        m, c = np.polyfit(z_values, x_values, 1)  # Fit a line to the lane points, treating z as the independent variable and x as the dependent variable. This will give us the slope (m) and intercept (c) of the line equation in the form x = m*z + c.
        return m, c 
    
    def check_lines_sign(self, left_line_eq, right_line_eq):
        # Implement logic to check signs of left and right line equations to determine track direction. This can be used to assist in determining the track direction and to assist in switching between different navigation strategies based on the distance to the goal.
        # By checking the signs of the slopes of the left and right line equations, we can determine the track direction. For example, if the slope of the left lane line is positive and the

        # Right line should have a positive gradient
        # Left line should have a negative gradient
        if left_line_eq is None or right_line_eq is None:
            return None  # Cannot determine track direction without both line equations
        
        left_slope = left_line_eq[0]
        right_slope = right_line_eq[0]

        if left_slope < 0 and right_slope > 0:
            return 'forward'
        elif left_slope > 0 and right_slope < 0:
            return 'backward'
        elif left_slope < 0 and right_slope < 0:
            # Move heading towards the left lane to try to get a better view of the track direction
            return 'left'
        elif left_slope > 0 and right_slope > 0:
            # Move heading towards the right lane to try to get a better view of the track direction
            return 'right'
        
    

    def lane_timer_callback(self):
        # This timer callback will be called periodically to compute the centreline and desired heading based on the latest LiDAR and lane detection data. It will then publish the appropriate cmd_vel to navigate towards the goal.
        
        self.get_logger().info('Running lane_timer_callback to compute centreline and desired heading. Checking check_lines_sign: ' + str(self.check_lines_sign(self.get_line_lane_equation(self.left_lane), self.get_line_lane_equation(self.right_lane))))

        

        left_eq = self.get_line_lane_equation(self.left_lane)
        right_eq = self.get_line_lane_equation(self.right_lane)

        lane_msg = LaneParameters()
        lane_msg.left_lane_gradient = left_eq[0] if left_eq else 0.0
        lane_msg.left_lane_x_intercept = left_eq[1] if left_eq else 0.0
        lane_msg.right_lane_gradient = right_eq[0] if right_eq else 0.0
        lane_msg.right_lane_x_intercept = right_eq[1] if right_eq else 0.0
        lane_msg.confidence = 1.0 if left_eq and right_eq else 0.5
        self.lane_parameters_publisher.publish(lane_msg)

        rec = NavigationRecommendation()
        rec.source = "track_follower"
        rec.valid = True

        direction = self.check_lines_sign(left_eq, right_eq)

        if direction == "forward":
            rec.reason = "both_lanes_forward"
            rec.linear_x = 0.5
            rec.angular_z = 0.0

        elif direction == "left":
            rec.reason = "both_lanes_suggest_left"
            rec.linear_x = 0.25
            rec.angular_z = 0.4

        elif direction == "right":
            rec.reason = "both_lanes_suggest_right"
            rec.linear_x = 0.25
            rec.angular_z = -0.4

        elif left_eq is not None and right_eq is None:
            # Only left lane visible.
            # Stay away from left lane, bias slightly right.
            rec.reason = "only_left_lane_visible"
            rec.linear_x = 0.25
            rec.angular_z = -0.25

        elif right_eq is not None and left_eq is None:
            # Only right lane visible.
            # Stay away from right lane, bias slightly left.
            rec.reason = "only_right_lane_visible"
            rec.linear_x = 0.25
            rec.angular_z = 0.25

        else:
            # No lane info. Let FTG/behaviour tree take over.
            rec.reason = "no_lanes_visible"
            rec.valid = False
            rec.linear_x = 0.0
            rec.angular_z = 0.0

        self.behaviour_tree_publisher.publish(rec)

        # Currently disabled this algorithm
        if self.temp_lane_timer_cb_test:
            # No centre line if we don't have both left and right lane points, so we can't compute a desired heading or navigate towards the goal without a centreline to follow. We will need to wait until we have both left and right lane points before we can compute the centreline and desired heading.
            if not self.left_lane or not self.right_lane:
                return
            
            centreline = self.compute_centreline()

            if not centreline:
                return
            
            desired_offset = self.track_width / 2.0  # Desired offset from the centreline to follow, this can be adjusted based on the desired position of the robot on the track (e.g. closer to the left or right lane). For now, we will aim to follow the centreline, so the desired offset is half the track width.
            # print(f"self.left_lane: {self.left_lane}")
            # print(f"self.right_lane: {self.right_lane}")

            # Check if we only see one of the lanes
            # if len(self.left_lane) > len(self.right_lane) * self.one_lane_threshold_factor:
            #     print("Only left lane detected, cannot compute centreline.")
            #     avg_x = np.mean([point[0] for point in self.left_lane])

            #     error = desired_offset - abs(avg_x)  # Compute the error based on the average x position of the detected lane points and the desired offset from the centreline. This is a simple proportional control approach where we compute the error as the difference between the desired offset and the actual offset based on the detected lane points, and we can use this error to compute a steering command to try to maintain the desired offset from the lane. We can adjust this to include a more sophisticated control approach if needed, such as a PID controller or a pure pursuit controller.

            #     steering = self.K_steering * error  # Compute the steering command based on the error and the steering gain. This will determine how aggressively the robot tries to correct its position based on the detected lane points.
            #     print(f"{steering=:.2f}, {error=:.2f}, {avg_x=:.2f}")
            #     return
            # elif len(self.right_lane) > len(self.left_lane) * self.one_lane_threshold_factor:
            #     print("Only right lane detected, cannot compute centreline.")
            #     avg_x = np.mean([point[0] for point in self.right_lane])
            #     error = desired_offset - abs(avg_x)  # Compute the error based on the average x position of the detected lane points and the desired offset from the centreline.

            #     steering = self.K_steering * error  # Compute the steering command based on the error and the steering gain.
            #     print(f"{steering=:.2f}, {error=:.2f}, {avg_x=:.2f}")
            #     return
            
            # Need enough centreline points
            if len(centreline) < 2:
                return

            # Smooth direction tracking
            start_idx = min(5, len(centreline) - 2)
            end_idx = min(10, len(centreline) - 1)

            dx_total = 0.0
            dz_total = 0.0
            count = 0

            for i in range(start_idx, end_idx):
                x1, y1, z1 = centreline[i]
                x2, y2, z2 = centreline[i + 1]

                dx_total += (x2 - x1)
                dz_total += (z2 - z1)
                count += 1

            if count == 0:
                return

            desired_heading = math.atan2(dx_total, dz_total)

            # Small correction toward centreline position
            target_idx = min(8, len(centreline) - 1)
            target_x, _, target_z = centreline[target_idx]

            position_error = math.atan2(target_x, target_z)

            # Blend both
            steering_error = desired_heading + 0.5 * position_error

            if self.test_track_follower_itself:
                cmd = Twist()
                cmd.linear.x = 0.5  # Set a constant forward speed, this can be adjusted based on the distance to the target or other factors
                cmd.angular.z = -1.0 * steering_error  # Proportional control for steering based on the error to the target
                

                self.cmd_vel_pub.publish(cmd)

            else:
                # Sends behaviour_tree the computed centreline and desired heading to assist in determining the track direction and goal location, and to assist in switching between different navigation strategies based on the distance to the goal. The behaviour tree will then use this information to determine which navigation strategy to use (e.g. follow the gap, goal navigation, etc.) and to compute the appropriate cmd_vel to publish to navigate towards the goal.
                
                navigation_recommendation = NavigationRecommendation()
                navigation_recommendation.source = 'track_follower'
                navigation_recommendation.valid = True
                navigation_recommendation.reason = 'centreline_computed'
                navigation_recommendation.linear_x = 0.5  # This can be adjusted based on the distance to the target or other factors
                navigation_recommendation.angular_z = -1.0 * steering_error  # Proportional control for steering based on the error to the target
                self.behaviour_tree_publisher.publish(navigation_recommendation)

            
            # print(f"The centreline points are: {centreline}")
            # print(f"Computed centreline with {len(centreline)} points.")
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

        left_line_equation = self.get_line_lane_equation(self.left_lane)
        right_line_equation = self.get_line_lane_equation(self.right_lane)

        print(f"len(self.left_lane): {len(self.left_lane)}. Left Lane Equation: x = {left_line_equation[0]} * z + {left_line_equation[1]}" if left_line_equation else "No left lane points to compute line equation.")
        print(f"len(self.right_lane): {len(self.right_lane)}. Right Lane Equation: x = {right_line_equation[0]} * z + {right_line_equation[1]}" if right_line_equation else "No right lane points to compute line equation.")

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