from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    px4_dir = '/opt/PX4-Autopilot'
    px4_build_dir = os.path.join(px4_dir, 'build/px4_sitl_default')

    # Start MicroXRCE-DDS Agent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen'
    )

    # Start PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=[
            os.path.join(px4_build_dir, 'bin/px4'),
            '-d',
            os.path.join(px4_build_dir, 'etc'),
        ],
        name='px4_sitl',
        output='screen',
        cwd=px4_build_dir,
        additional_env={
            'PX4_SIM_MODEL': 'gazebo-classic_iris',
            'PX4_SYS_AUTOSTART': '10015',
        }
    )

    return LaunchDescription([
        micro_xrce_agent,
        px4_sitl,
    ])
