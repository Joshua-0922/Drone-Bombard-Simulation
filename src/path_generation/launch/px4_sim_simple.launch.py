from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():
    px4_dir = '/opt/PX4-Autopilot'

    # Launch arguments
    enable_path_generation = LaunchConfiguration('enable_path_generation')

    declare_enable_path = DeclareLaunchArgument(
        'enable_path_generation',
        default_value='false',
        description='Launch circle_path node'
    )

    # Set ROS version for PX4
    set_ros_version = SetEnvironmentVariable(
        name='ROS_VERSION',
        value='2'
    )

    # 1. Start MicroXRCE-DDS Agent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen',
    )

    # 2. Start PX4 SITL with Gazebo using make command
    # This uses PX4's built-in Gazebo integration
    px4_gazebo = ExecuteProcess(
        cmd=[
            'make',
            'px4_sitl',
            'gazebo-classic_iris__empty'
        ],
        name='px4_gazebo',
        output='screen',
        cwd=px4_dir,
        additional_env={
            'ROS_VERSION': '2',
            'HEADLESS': '1',  # Run without GUI
        }
    )

    # 3. Optional: Launch path_generation node (after 15 seconds delay)
    circle_path_node = Node(
        package='path_generation',
        executable='circle_path',
        name='circle_path_node',
        output='screen',
        condition=IfCondition(enable_path_generation),
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        # Arguments
        declare_enable_path,

        # Environment
        set_ros_version,

        # Processes
        micro_xrce_agent,
        px4_gazebo,
        circle_path_node,
    ])
