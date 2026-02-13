import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Vision Node (CMake 패키지)
        Node(
            package='vision_detection',
            # 중요: CMakeLists.txt 설정에 따라 .py가 붙을 수도, 안 붙을 수도 있습니다.
            # 만약 "executable not found" 에러가 나면 '.py'를 붙여보세요.
            executable='xmarker_detector', 
            name='xmarker_detector',
            output='screen',
            parameters=[
                {'inference_rate': 10.0},
                {'model_path': '/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/weights/best.pt'}
            ]
        ),

        # 2. Mission Manager Node (Python 패키지)
        Node(
            package='mission_manager', # 패키지 이름 (setup.py의 package_name)
            executable='mission_manager_node', # console_scripts에 적힌 이름
            name='mission_manager',
            output='screen'
        ),

        # 3. Drone Controller Node (Python 패키지)
        Node(
            package='drone_controller', # 패키지 이름
            executable='controller',    # setup.py에서 'controller =' 라고 정의했으므로 이게 실행 이름임
            name='drone_controller',
            output='screen'
        ),

        # 4. RL Navigation Node (추가됨)
        Node(
            package='rl_navigation', 
            executable='rl_navigation_node',  # setup.py의 'rl_navigation_node ='
            name='rl_navigation',
            output='screen'
        ),

        # 5. Drop Calculator Node - 심판 (추가됨)
        Node(
            package='drop_calculator', 
            executable='calculator',          # setup.py의 'calculator ='
            name='drop_calculator',
            output='screen'
        ),
    ])