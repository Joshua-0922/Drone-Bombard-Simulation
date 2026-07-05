"""
infra_rad.launch.py
===================
Persistent infrastructure layer for RAD v1 training.

기존 infra.launch.py 와 거의 동일 — 변경 사항:
  • 코드 측 변경 없음 (인프라는 RAD/v8 공유 가능)
  • 단 RAD 학습 시 헷갈리지 않도록 별도 launch 파일 유지

Start ONCE per RAD training session. RAD env (drone_drop_env_rad.py) 의
reset() 가 episode_rad.launch.py 로 mission nodes 만 재시작.

Launch order (infra.launch.py 와 동일):
  t=0s  : MicroXRCEAgent
  t=0s  : Gazebo Harmonic (x_marker_world.sdf, RAD 도 동일 world)
  t=10s : ros_gz_bridge
  t=20s : PX4 SITL
  t=22s : xmarker_detector (RAD 는 use_vision=false 라 dead, 단 launch arg 로 켤 수 있음)
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
    px4_dir = "/opt/PX4-Autopilot"
    px4_gz_models = f"{px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds = f"{px4_dir}/Tools/simulation/gz/worlds"

    gz_resource_path = ":".join([models_dir, px4_gz_models, px4_gz_worlds])

    bridge_config = os.path.join(
        os.path.dirname(__file__), "..", "config", "ros_gz_bridge.yaml")

    # Launch args
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="true",
        description="Gazebo headless (default true for RL)")
    vision_arg = DeclareLaunchArgument(
        "enable_vision", default_value="false",
        description="YOLO detector (default false — RAD 는 vision off)")

    headless = LaunchConfiguration("headless")
    enable_vision = LaunchConfiguration("enable_vision")

    # [1] uXRCE-DDS Agent
    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_dds_agent",
        output="screen",
    )

    # [2] Gazebo Harmonic (x_marker_world.sdf — RAD/v8 공유)
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

    # [3] ros_gz_bridge (t=10s)
    ros_gz_bridge = TimerAction(
        period=10.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        )],
    )

    # [4] PX4 SITL (t=20s)
    _px4_gz_bridge_dir = (
        f"{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_bridge"
    )
    _px4_bin = f"{px4_dir}/build/px4_sitl_default/bin/px4"

    px4_sitl = TimerAction(
        period=20.0,
        actions=[ExecuteProcess(
            cmd=["bash", "-c",
                 f"cd {_px4_gz_bridge_dir} && "
                 f"PX4_GZ_STANDALONE=1 "
                 f"PX4_GZ_WORLD=x_marker_world "
                 f"PX4_SIM_MODEL=gz_x500_bombard "
                 f"PX4_SIM_SPEED_FACTOR=1 "
                 f"PX4_GZ_MODEL_POSE='0,0,5,0,0,0' "
                 f"GZ_SIM_RESOURCE_PATH={gz_resource_path} "
                 f"{_px4_bin}"],
            name="px4_sitl",
            output="screen",
        )],
    )

    # [5] YOLO vision (t=22s, default off)
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

    return LaunchDescription([
        headless_arg,
        vision_arg,
        micro_xrce_agent,
        gz_sim_gui,
        gz_sim_headless,
        ros_gz_bridge,
        px4_sitl,
        xmarker_detector,
        LogInfo(msg=[
            "\n=== RAD v1 Infra Layer Started ===\n",
            "  Gazebo + MicroXRCEAgent + ros_gz_bridge + PX4 SITL (persistent) + YOLO (off)\n",
            "  Keep running for the full RAD training session.\n",
            "  Episode resets restart ONLY mission nodes via episode_rad.launch.py.\n",
        ]),
    ])
