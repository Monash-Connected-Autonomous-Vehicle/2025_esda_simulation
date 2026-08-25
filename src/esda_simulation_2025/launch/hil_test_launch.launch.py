#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'esda_simulation_2025'

    # ------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_teleop = LaunchConfiguration('launch_teleop')


    # ------------------------------------------------------------
    # Robot State Publisher
    #
    # Loads the URDF and publishes the robot link transforms.
    # No Gazebo, no LiDAR.
    # ------------------------------------------------------------

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'false',
            'use_lidar': 'false',
        }.items()
    )


    # ------------------------------------------------------------
    # Joint State Publisher
    #
    # Publishes default positions for movable joints so that
    # robot_state_publisher can publish transforms for the wheels.
    # ------------------------------------------------------------

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'publish_default_positions': True,
                'rate': 30.0,
            }
        ]
    )


    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(launch_rviz)
    )


    # ------------------------------------------------------------
    # WASD Teleop
    #
    # This script publishes:
    #
    #   geometry_msgs/msg/Twist
    #            ↓
    #         /cmd_vel
    #
    # xterm is used because your teleop script reads keyboard
    # input directly from stdin.
    # ------------------------------------------------------------

    teleop_script = 'src/esda_simulation_2025/scripts/teleop_wasd.py'

    teleop = ExecuteProcess(
        cmd=[
            'xterm',
            '-T',
            'ESDA WASD Teleop',
            '-e',
            'python3',
            teleop_script
        ],
        output='screen',
        condition=IfCondition(launch_teleop)
    )


    # ------------------------------------------------------------
    # Throttle Publisher
    #
    # This script should subscribe to:
    #
    #   /cmd_vel
    #
    # and publish:
    #
    #   /esda_throttle_topic
    #
    # ------------------------------------------------------------

    throttle_script = 'src/esda_simulation_2025/scripts/throttle_publisher.py'

    throttle_publisher = ExecuteProcess(
        cmd=[
            'python3',
            throttle_script
        ],
        output='screen'
    )

    odom_script = 'src/esda_simulation_2025/scripts/cmd_vel_odometry.py'

    cmd_vel_odometry = ExecuteProcess(
        cmd=[
            'python3',
            odom_script
        ],
        output='screen'
    )

    # ------------------------------------------------------------
    # Launch Description
    # ------------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'launch_teleop',
            default_value='true'
        ),

        # Robot model
        rsp,

        # Joint states for wheel transforms
        joint_state_publisher,

        # Convert /cmd_vel -> throttle
        throttle_publisher,

        cmd_vel_odometry,

        # RViz
        rviz,

        # WASD keyboard control
        teleop,
    ])