"""
drone_mission.launch.py
========================
Single launch file for the full Gazebo Harmonic drone simulation.

Launch order:
  t=0s  : MicroXRCEAgent  (PX4 <-> ROS2 DDS bridge)
  t=0s  : gz sim          (Gazebo Harmonic world)
  t=5s  : PX4 SITL        (connects to running gz, spawns x500_bombard)
  t=8s  : ros_gz_bridge   (bridges gz-transport -> ROS2 topics)
  t=12s : xmarker_detector (YOLO vision node)
  t=0s  : mission nodes   (wait on ROS2 topics; start immediately)

Prerequisites (inside drone-bombard-harmonic container):
  1. Build PX4 for Harmonic target (one-time):
       cd /opt/PX4-Autopilot
       DONT_RUN=1 make px4_sitl gz_x500
  2. Build ROS2 workspace:
       cd /workspace/ros2_ws && colcon build && source install/setup.bash

Run:
  ros2 launch mission_manager drone_mission.launch.py
  ros2 launch mission_manager drone_mission.launch.py headless:=true
  ros2 launch mission_manager drone_mission.launch.py enable_vision:=false

Launch arguments:
  headless      Run Gazebo without GUI (default: false)
  enable_vision Launch xmarker_detector YOLO node (default: true)
  rl_mode       Skip rl_navigation_node so RL env drives velocity directly (default: false)
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
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    gz_env = {
        "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        "PX4_GZ_STANDALONE": "1",
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
    }

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

    rl_mode_arg = DeclareLaunchArgument(
        "rl_mode", default_value="false",
        description="Skip rl_navigation_node; RL env drives velocity directly")

    headless = LaunchConfiguration("headless")
    enable_vision = LaunchConfiguration("enable_vision")
    rl_mode = LaunchConfiguration("rl_mode")

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
    gz_sim_gui = ExecuteProcess(
        cmd=["bash", "-c",
             f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
             f"gz sim -r {worlds_dir}/x_marker_world.sdf"],
        name="gz_sim",
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["bash", "-c",
             f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
             f"gz sim -r -s {worlds_dir}/x_marker_world.sdf"],
        name="gz_sim_headless",
        output="screen",
        condition=IfCondition(headless),
    )

    # ---------------------------------------------------------------------------
    # [3] PX4 SITL (t=5s — wait for Gazebo)
    # ---------------------------------------------------------------------------
    px4_sitl = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=["bash", "-c",
                 f"cd {px4_dir} && "
                 f"PX4_GZ_STANDALONE=1 "
                 f"PX4_GZ_MODEL=x500_bombard "
                 f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
                 f"make px4_sitl gz_x500"],
            name="px4_sitl",
            output="screen",
        )],
    )

    # ---------------------------------------------------------------------------
    # [4] ros_gz_bridge (t=8s — wait for Gazebo + PX4 + sensors)
    # ---------------------------------------------------------------------------
    ros_gz_bridge = TimerAction(
        period=8.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        )],
    )

    # ---------------------------------------------------------------------------
    # [5] YOLO vision node (t=12s — wait for bridge + camera)
    # ---------------------------------------------------------------------------
    xmarker_detector = TimerAction(
        period=12.0,
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
    # [6] Mission nodes (start immediately; block on ROS2 topic availability)
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
        headless_arg,
        vision_arg,
        rl_mode_arg,
        micro_xrce_agent,
        gz_sim_gui,
        gz_sim_headless,
        px4_sitl,
        ros_gz_bridge,
        xmarker_detector,
        mission_manager,
        drone_controller,
        rl_navigation,
        drop_calculator,
        LogInfo(msg=[
            "\n=== Drone Mission Launch (Gazebo Harmonic) ===\n",
            f"  World:       {worlds_dir}/x_marker_world.sdf\n",
            f"  Model:       x500_bombard\n",
            f"  Models path: {gz_resource_path}\n",
            f"  Bridge cfg:  {bridge_config}\n",
        ]),
    ])
