from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    loop_waypoints_node = Node(
        package='esda_simulation_2025',
        executable='loop_waypoints.py',
        name='loop_waypoints_node',
        output='screen',
        parameters=[
            {'loop_forever': True},
            {'pause_seconds': 2.0},
        ]
    )

    return LaunchDescription([
        loop_waypoints_node,
    ])