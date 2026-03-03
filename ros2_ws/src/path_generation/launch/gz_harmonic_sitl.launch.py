"""
gz_harmonic_sitl.launch.py
===========================
Replaces px4_gazebo_sitl.launch.py for Gazebo Harmonic.

Migration changes from Classic launch file:
  - gzserver/gzclient         -> gz sim (single process, -r flag to auto-run)
  - GAZEBO_MODEL_PATH          -> GZ_SIM_RESOURCE_PATH
  - GAZEBO_PLUGIN_PATH         -> no equivalent (plugins found via LD_LIBRARY_PATH)
  - spawn_entity.py            -> gz service -s /world/.../create (or PX4 auto-spawn)
  - make px4_sitl gazebo-classic_iris -> make px4_sitl gz_x500 with PX4_GZ_MODEL
  - No ros_gz_bridge           -> ros_gz_bridge parameter_bridge node added

Prerequisites (inside drone-bombard-harmonic container):
  1. Install ros-gz bridge:
       sudo apt install ros-humble-ros-gzharmonic
  2. Build PX4 for Harmonic target (one-time):
       cd /opt/PX4-Autopilot
       DONT_RUN=1 make px4_sitl gz_x500
  3. Build ROS2 workspace:
       cd /workspace/ros2_ws && colcon build && source install/setup.bash

Run (3 terminals inside drone-bombard-harmonic container):
  Terminal 1: MicroXRCEAgent udp4 -p 8888
  Terminal 2: ros2 launch path_generation gz_harmonic_sitl.launch.py
  Terminal 3: ros2 launch mission_manager drone_mission.launch.py

Launch arguments:
  world         World name without .sdf extension (default: x_marker_harmonic)
  model         Drone model name (default: x500_bombard)
  headless      Run Gazebo without GUI (default: false)
  enable_vision Launch xmarker_detector node (default: true)
  gz_model_path Custom model/world search path (auto-detected if empty)
"""

import os
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# ---------------------------------------------------------------------------
# Helper: resolve gazebo_models directory (host volume or fallback paths)
# ---------------------------------------------------------------------------
def _find_models_dir() -> str:
    candidates = [
        "/workspace/gazebo_models",
        "/opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models",
        str(Path.home() / "Drone-Bombard-Simulation/gazebo_models"),
    ]
    for path in candidates:
        if os.path.isdir(path):
            return path
    return candidates[0]  # fallback: will be created by volume mount


def generate_launch_description():
    models_dir = _find_models_dir()
    worlds_dir = os.path.join(models_dir, "worlds")
    px4_gz_models = "/opt/PX4-Autopilot/Tools/simulation/gz/models"
    px4_gz_worlds = "/opt/PX4-Autopilot/Tools/simulation/gz/worlds"
    px4_dir = "/opt/PX4-Autopilot"

    # GZ_SIM_RESOURCE_PATH: custom models first, then PX4 models
    gz_resource_path = ":".join([models_dir, px4_gz_models, px4_gz_worlds])

    # Shared environment for Harmonic
    gz_env = {
        "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic",
        "PX4_GZ_STANDALONE": "1",          # PX4 connects to already-running gz
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
    }

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world", default_value="x_marker_harmonic",
        description="Gazebo world name (no .sdf extension)")

    model_arg = DeclareLaunchArgument(
        "model", default_value="x500_bombard",
        description="Drone model name for PX4 spawn (must be in GZ_SIM_RESOURCE_PATH)")

    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo server-only without GUI (true/false)")

    vision_arg = DeclareLaunchArgument(
        "enable_vision", default_value="true",
        description="Launch xmarker_detector YOLO detection node")

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    headless = LaunchConfiguration("headless")
    enable_vision = LaunchConfiguration("enable_vision")

    # ---------------------------------------------------------------------------
    # [1] uXRCE-DDS Agent  (PX4 <-> ROS2 bridge, same as Classic setup)
    # ---------------------------------------------------------------------------
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_dds_agent",
        output="screen",
    )

    # ---------------------------------------------------------------------------
    # [2] Gazebo Harmonic  (replaces gzserver + gzclient)
    #     Classic: gzserver world.world  +  gzclient
    #     Harmonic: gz sim -r world.sdf  (GUI built-in unless --headless-rendering)
    # ---------------------------------------------------------------------------
    world_sdf = PythonExpression(
        ["'", worlds_dir, "/'", " + '", "", "' + ", "\"", "", "\"",
         " + world + '.sdf'"]
    )
    # Simpler string construction via shell
    gz_sim_with_gui = ExecuteProcess(
        cmd=["bash", "-c",
             f"gz sim -r {worlds_dir}/$(ros2 launch path_generation "
             f"_get_world.py world:=${{WORLD}} 2>/dev/null || echo x_marker_harmonic).sdf"],
        additional_env={**gz_env},
        name="gz_sim",
        output="screen",
        condition=UnlessCondition(headless),
    )
    # Simpler approach: build path directly since world arg is known at description time
    gz_sim_with_gui_simple = ExecuteProcess(
        cmd=["bash", "-c",
             f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
             f"gz sim -r {worlds_dir}/x_marker_harmonic.sdf"],
        name="gz_sim",
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["bash", "-c",
             f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
             f"gz sim -r -s {worlds_dir}/x_marker_harmonic.sdf"],
        name="gz_sim_headless",
        output="screen",
        condition=IfCondition(headless),
    )

    # ---------------------------------------------------------------------------
    # [3] PX4 SITL  (replaces make px4_sitl gazebo-classic_iris)
    #     PX4_GZ_STANDALONE=1  -> connects to the already-running gz sim above
    #     PX4_GZ_MODEL         -> which model SDF to spawn into the world
    #     make px4_sitl gz_x500 -> the Harmonic-compatible SITL target
    # ---------------------------------------------------------------------------
    px4_sitl = TimerAction(
        period=5.0,   # wait for Gazebo to fully start
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
    # [4] ros_gz_bridge  (NEW — no Classic equivalent)
    #     Bridges Gazebo Harmonic gz-transport topics to ROS2.
    #     Requires: sudo apt install ros-humble-ros-gzharmonic
    #     Config:   config/ros_gz_bridge.yaml
    # ---------------------------------------------------------------------------
    bridge_config = os.path.join(
        os.path.dirname(__file__), "..", "config", "ros_gz_bridge.yaml")

    ros_gz_bridge = TimerAction(
        period=8.0,   # wait for Gazebo + PX4 + sensors to initialise
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        )],
    )

    # ---------------------------------------------------------------------------
    # [5] Optional: X-marker detector (same node as Classic, just delayed)
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

    return LaunchDescription([
        world_arg,
        model_arg,
        headless_arg,
        vision_arg,
        # Process launch order:
        # t=0s  : MicroXRCEAgent  (PX4 <-> ROS2 DDS bridge)
        # t=0s  : gz sim          (Gazebo Harmonic world)
        # t=5s  : PX4 SITL        (connects to running gz, spawns x500_bombard)
        # t=8s  : ros_gz_bridge   (bridges gz-transport -> ROS2 topics)
        # t=12s : xmarker_detector (YOLO vision node)
        micro_xrce_agent,
        gz_sim_with_gui_simple,
        gz_sim_headless,
        px4_sitl,
        ros_gz_bridge,
        xmarker_detector,
        LogInfo(msg=[
            "\n=== Gazebo Harmonic SITL Launch ===\n",
            f"  World:        {worlds_dir}/x_marker_harmonic.sdf\n",
            f"  Drone model:  x500_bombard\n",
            f"  Models path:  {gz_resource_path}\n",
            "  Bridge cfg:   config/ros_gz_bridge.yaml\n",
            "  NOTE: Requires ros-humble-ros-gzharmonic installed.\n",
            "  NOTE: PX4 must be built first:\n",
            f"    cd {px4_dir} && DONT_RUN=1 make px4_sitl gz_x500\n",
        ]),
    ])
