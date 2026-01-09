import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'drop_calculator'
    
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'drop_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='calculator', # setup.py entry point
            name='drop_calculator_node',
            output='screen',
            parameters=[config_file]
        )
    ])