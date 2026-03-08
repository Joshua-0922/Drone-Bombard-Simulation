"""
test_mission.launch.py
======================
Test launch file for the drone simulation.  Identical to drone_mission.launch.py
but replaces rl_navigation_node with the simpler simple_drop node (confidence
threshold 0.5, no RL logic).

Run:
  ros2 launch mission_manager test_mission.launch.py
  ros2 launch mission_manager test_mission.launch.py headless:=true
  ros2 launch mission_manager test_mission.launch.py enable_vision:=false

Launch arguments:
  headless      Run Gazebo without GUI (default: false)
  enable_vision Launch xmarker_detector YOLO node (default: true)
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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
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
    worlds_dir = os.path.join(models_dir, "worlds")
    px4_gz_models = "/opt/PX4-Autopilot/Tools/simulation/gz/models"
    px4_gz_worlds = "/opt/PX4-Autopilot/Tools/simulation/gz/worlds"
    px4_dir = "/opt/PX4-Autopilot"

    gz_resource_path = ":".join([models_dir, px4_gz_models, px4_gz_worlds])

    bridge_config = os.path.join(
        os.path.dirname(__file__), "..", "config", "ros_gz_bridge.yaml")

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo server-only without GUI (true/false)")

    vision_arg = DeclareLaunchArgument(
        "enable_vision", default_value="true",
        description="Launch xmarker_detector YOLO detection node")

    headless = LaunchConfiguration("headless")
    enable_vision = LaunchConfiguration("enable_vision")

    # ---------------------------------------------------------------------------
    # [1] uXRCE-DDS Agent
    # ---------------------------------------------------------------------------
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_dds_agent",
        output="screen",
    )

    # ---------------------------------------------------------------------------
    # [2] Gazebo Harmonic
    # ---------------------------------------------------------------------------
    _gz_env_prefix = (
        "mkdir -p /tmp/runtime-root && "
        "export XDG_RUNTIME_DIR=/tmp/runtime-root && "
        f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
    )
    gz_sim_gui = ExecuteProcess(
        cmd=["bash", "-c",
             _gz_env_prefix +
             f"gz sim -r {worlds_dir}/x_marker_world.sdf"],
        name="gz_sim",
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["bash", "-c",
             _gz_env_prefix +
             f"gz sim -r -s {worlds_dir}/x_marker_world.sdf"],
        name="gz_sim_headless",
        output="screen",
        condition=IfCondition(headless),
    )

    # ---------------------------------------------------------------------------
    # [3] PX4 SITL (t=12s)
    # ---------------------------------------------------------------------------
    _px4_gz_bridge_dir = (
        f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_bridge"
    )
    _px4_bin = f"{px4_dir}/build/px4_sitl_default/bin/px4"
    px4_sitl = TimerAction(
        period=12.0,
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
    # [4] ros_gz_bridge (t=16s)
    # ---------------------------------------------------------------------------
    ros_gz_bridge = TimerAction(
        period=16.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        )],
    )

    # ---------------------------------------------------------------------------
    # [5] YOLO vision node (t=22s)
    # ---------------------------------------------------------------------------
    xmarker_detector = TimerAction(
        period=22.0,
        actions=[Node(
            package="vision_detection",
            executable="xmarker_detector",
            name="xmarker_detector",
            output="screen",
            parameters=[
                {"inference_rate": 10.0},
                {"model_path":
                    "/workspace/ros2_ws/yolo_workspace/runs/train/"
                    "drone_bombard_train2/weights/best.pt"},
            ],
            condition=IfCondition(enable_vision),
        )],
    )

    # ---------------------------------------------------------------------------
    # [6] Mission nodes
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
    simple_drop = Node(
        package="rl_navigation",
        executable="simple_drop",
        name="simple_drop",
        output="screen",
    )
    drop_calculator = Node(
        package="drop_calculator",
        executable="calculator",
        name="drop_calculator",
        output="screen",
        parameters=[{"x_marker_x": 11.0}, {"x_marker_y": 10.0}],
    )

    return LaunchDescription([
        headless_arg,
        vision_arg,
        micro_xrce_agent,
        gz_sim_gui,
        gz_sim_headless,
        px4_sitl,
        ros_gz_bridge,
        xmarker_detector,
        mission_manager,
        drone_controller,
        simple_drop,
        drop_calculator,
        LogInfo(msg=[
            "\n=== Test Mission Launch (simple_drop, confidence > 0.5) ===\n",
            f"  World:       {worlds_dir}/x_marker_world.sdf\n",
            f"  Model:       x500_bombard\n",
            f"  Models path: {gz_resource_path}\n",
            f"  Bridge cfg:  {bridge_config}\n",
        ]),
    ])
