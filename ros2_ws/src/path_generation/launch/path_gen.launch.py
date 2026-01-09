import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'path_generation'
    
    # Config 파일 경로 찾기
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='generator', # setup.py entry point 이름
            name='path_generation_node',
            output='screen',
            parameters=[config_file] # 여기서 YAML 로드
        )
    ])