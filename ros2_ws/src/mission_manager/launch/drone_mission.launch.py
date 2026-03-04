from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Mission Manager Node
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen'
        ),

        # 2. Drone Controller Node
        Node(
            package='drone_controller',
            executable='controller',
            name='drone_controller',
            output='screen'
        ),

        # 3. RL Navigation Node
        Node(
            package='rl_navigation',
            executable='rl_navigation_node',
            name='rl_navigation',
            output='screen'
        ),

        # 4. Drop Calculator Node
        Node(
            package='drop_calculator',
            executable='calculator',
            name='drop_calculator',
            output='screen',
            parameters=[{'x_marker_x': 11.0}, {'x_marker_y': 10.0}]
        ),
    ])