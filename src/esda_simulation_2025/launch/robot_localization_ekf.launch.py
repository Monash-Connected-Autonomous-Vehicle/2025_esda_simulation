import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the path to your configuration file
    # Change 'your_robot_package' to match your actual package folder name
    ekf_config_path = os.path.join(
        get_package_share_directory('esda_simulation_2025'),
        'config',
        'ekf.yaml'
    )


    # 2. Define the node execution parameters
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path] # This maps your yaml settings into the node
    )

    # 3. Return the launch system instructions
    return LaunchDescription([
        ekf_node
    ])
