"""
episode.launch.py
=================
Episode-level processes for RL training.

Killed and restarted by DroneDropEnv.reset() between episodes.
Assumes infra.launch.py is already running (Gazebo + ros_gz_bridge + YOLO).

Launch order:
  t=0s : mission_manager, drone_controller, drop_calculator (block on PX4 topics)
  t=2s : PX4 SITL (connects to already-running Gazebo, spawns drone)

Launch arguments:
  rl_mode  Skip rl_navigation_node; RL env drives velocity directly (default: true)
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _find_models_dir() -> str:
    candidates = [
        "/workspace/gazebo_models",
        "/opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models",
        str(Path.home() / "Drone-Bombard-Simulation/gazebo_models"),
    ]
    for path in candidates:
        if os.path.isdir(path):
            return path
    return candidates[0]


def generate_launch_description():
    models_dir = _find_models_dir()
    px4_dir = "/opt/PX4-Autopilot"
    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"
    gz_resource_path = ":".join([models_dir, px4_gz_models, px4_gz_worlds])

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    rl_mode_arg = DeclareLaunchArgument(
        "rl_mode", default_value="true",
        description="Skip rl_navigation_node; RL env drives velocity directly")

    rl_mode = LaunchConfiguration("rl_mode")

    # ---------------------------------------------------------------------------
    # [1] PX4 SITL (t=2s -- Gazebo is already running from infra.launch.py)
    #
    # Short 2s delay gives the gz world reset time to fully settle before PX4
    # connects and spawns the drone model.
    # ---------------------------------------------------------------------------
    _px4_gz_bridge_dir = (
        f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_bridge"
    )
    _px4_bin = f"{px4_dir}/build/px4_sitl_default/bin/px4"

    px4_sitl = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=["bash", "-c",
                 f"cd {_px4_gz_bridge_dir} && "
                 f"PX4_GZ_STANDALONE=1 "
                 f"PX4_GZ_WORLD=x_marker_world "
                 f"PX4_SIM_MODEL=gz_x500_bombard "
                 f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
                 f"{_px4_bin}"],
            name="px4_sitl",
            output="screen",
        )],
    )

    # ---------------------------------------------------------------------------
    # [2] Mission nodes -- start immediately, block on PX4 topic availability
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
        px4_sitl,
        mission_manager,
        drone_controller,
        rl_navigation,
        drop_calculator,
        LogInfo(msg=[
            "\n=== Episode Layer Started ===\n",
            "  PX4 SITL + mission nodes\n",
            "  This process group is restarted by DroneDropEnv.reset().\n",
        ]),
    ])
