"""
episode_rad.launch.py
=====================
Episode-level processes for RAD v1 training.

기존 episode.launch.py 와 차이:
  • mission_manager_rad (spawn yaw 랜덤 + cruise 1m/s 가속)
  • drop_calculator target = (4, 3) RAD design 명시
  • drone_controller 는 그대로 (v8/RAD 공유)

Killed and restarted by DroneDropEnvRAD.reset() between episodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rl_mode_arg = DeclareLaunchArgument(
        "rl_mode", default_value="true",
        description="Skip rl_navigation_node; RL env drives velocity directly")

    rl_mode = LaunchConfiguration("rl_mode")

    # --- Mission nodes (RAD specific) ---
    mission_manager = Node(
        package="mission_manager",
        executable="mission_manager_rad",   # RAD version (entry_point in setup.py)
        name="mission_manager_rad",
        output="screen",
        parameters=[
            {"target_enu_x": 4.0},
            {"target_enu_y": 3.0},
            {"target_altitude": 5.0},
            {"cruise_target_speed": 1.0},
            {"yaw_relative_range": 1.5708},
            {"spawn_yaw_random_enabled": True},
        ],
    )
    drone_controller = Node(
        package="drone_controller",
        executable="controller",
        name="drone_controller",
        output="screen",
    )
    # rl_navigation_node 는 RAD 학습 시 사용 안 함 (env 가 직접 velocity 발행)
    rl_navigation = Node(
        package="rl_navigation",
        executable="rl_navigation_node",
        name="rl_navigation",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", rl_mode, "' == 'false'"])),
    )
    # Drop calculator — RAD target (4, 3) 명시 (지면 z=0)
    drop_calculator = Node(
        package="drop_calculator",
        executable="calculator",
        name="drop_calculator",
        output="screen",
        parameters=[
            {"x_marker_x": 4.0},   # RAD: target_enu_x
            {"x_marker_y": 3.0},   # RAD: target_enu_y
        ],
    )

    return LaunchDescription([
        rl_mode_arg,
        mission_manager,
        drone_controller,
        rl_navigation,
        drop_calculator,
        LogInfo(msg=[
            "\n=== RAD v1 Episode Layer Started ===\n",
            "  mission_manager_rad (yaw rand, cruise 1m/s) + drone_controller + drop_calculator (target 4,3)\n",
            "  Restarted by DroneDropEnvRAD.reset().\n",
        ]),
    ])
