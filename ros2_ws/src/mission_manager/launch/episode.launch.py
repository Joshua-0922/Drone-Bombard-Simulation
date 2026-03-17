"""
episode.launch.py
=================
Episode-level processes for RL training.

Killed and restarted by DroneDropEnv.reset() between episodes.
Assumes infra.launch.py is already running (Gazebo + MicroXRCEAgent +
ros_gz_bridge + PX4 SITL + YOLO).

PX4 SITL is now persistent in infra.launch.py — NOT launched here.
This file only manages mission nodes that are safe to restart per episode.

Launch order:
  t=0s : mission_manager, drone_controller, drop_calculator

Launch arguments:
  rl_mode  Skip rl_navigation_node; RL env drives velocity directly (default: true)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    rl_mode_arg = DeclareLaunchArgument(
        "rl_mode", default_value="true",
        description="Skip rl_navigation_node; RL env drives velocity directly")

    rl_mode = LaunchConfiguration("rl_mode")

    # ---------------------------------------------------------------------------
    # [1] Mission nodes -- start immediately, block on PX4 topic availability
    # PX4 SITL is persistent in infra.launch.py; it is NOT launched here.
    # ---------------------------------------------------------------------------
    mission_manager = Node(
        package="mission_manager",
        executable="mission_manager_node",
        name="mission_manager",
        output="screen",
    )
    drone_controller = Node(
        package="drone_controller",
        executable="controller",
        name="drone_controller",
        output="screen",
    )
    rl_navigation = Node(
        package="rl_navigation",
        executable="rl_navigation_node",
        name="rl_navigation",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", rl_mode, "' == 'false'"])),
    )
    drop_calculator = Node(
        package="drop_calculator",
        executable="calculator",
        name="drop_calculator",
        output="screen",
        parameters=[{"x_marker_x": 11.0}, {"x_marker_y": 10.0}],
    )

    return LaunchDescription([
        rl_mode_arg,
        mission_manager,
        drone_controller,
        rl_navigation,
        drop_calculator,
        LogInfo(msg=[
            "\n=== Episode Layer Started ===\n",
            "  mission nodes only (PX4 SITL is persistent in infra.launch.py)\n",
            "  This process group is restarted by DroneDropEnv.reset().\n",
        ]),
    ])
