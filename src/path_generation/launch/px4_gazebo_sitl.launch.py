from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Paths
    px4_dir = '/opt/PX4-Autopilot'
    px4_build_dir = os.path.join(px4_dir, 'build/px4_sitl_default')
    gazebo_models = os.path.join(px4_dir, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models')
    gazebo_worlds = os.path.join(px4_dir, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds')
    gazebo_plugins = os.path.join(px4_build_dir, 'build_gazebo-classic')

    # Launch arguments
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    headless = LaunchConfiguration('headless')
    enable_path_generation = LaunchConfiguration('enable_path_generation')
    enable_vision = LaunchConfiguration('enable_vision')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world name (without .world extension)'
    )

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='iris_downward_depth_camera',
        description='PX4 vehicle model'
    )

    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    declare_enable_path = DeclareLaunchArgument(
        'enable_path_generation',
        default_value='false',
        description='Launch circle_path node'
    )

    declare_enable_vision = DeclareLaunchArgument(
        'enable_vision',
        default_value='false',
        description='Launch X-marker detection node'
    )

    # Environment variables
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_models
    )

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=gazebo_plugins
    )

    set_ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value=gazebo_plugins + ':' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    set_ros_version = SetEnvironmentVariable(
        name='ROS_VERSION',
        value='2'
    )

    # 1. Start MicroXRCE-DDS Agent (PX4-ROS2 bridge)
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        name='micro_xrce_agent',
        output='screen',
    )

    # 2. Start Gazebo server
    world_path = PathJoinSubstitution([gazebo_worlds, [world, '.world']])

    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        name='gazebo_server',
        output='screen',
        additional_env={'GAZEBO_MODEL_PATH': gazebo_models}
    )

    # 3. Start Gazebo client (GUI) - only if not headless
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        name='gazebo_client',
        output='screen',
        condition=UnlessCondition(headless)
    )

    # 4. Spawn Iris model using Gazebo ROS spawn_entity service (after delay)
    model_sdf = PathJoinSubstitution([gazebo_models, model, [model, '.sdf']])

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model,
            '-file', model_sdf,
            '-x', x,
            '-y', y,
            '-z', z,
        ],
        name='spawn_entity',
        output='screen'
    )

    # Delay spawn to ensure Gazebo is ready
    spawn_entity_delayed = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # 5. Start PX4 SITL (with delay to ensure MicroXRCE Agent is ready)
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
            'PX4_SIM_MODEL': ['gazebo-classic_', model],
            'PX4_SYS_AUTOSTART': '10015',
        }
    )

    px4_sitl_delayed = TimerAction(
        period=2.0,
        actions=[px4_sitl]
    )

    # 6. Optional: Launch path_generation node
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

    circle_path_delayed = TimerAction(
        period=10.0,
        actions=[circle_path_node]
    )

    # 7. Optional: Launch X-marker detector node
    xmarker_detector_node = Node(
        package='vision_detection',
        executable='xmarker_detector',
        name='xmarker_detector',
        output='screen',
        condition=IfCondition(enable_vision),
        parameters=[
            {'use_sim_time': True},
            {'model_path': '/workspace/ros2_ws/yolo_workspace/runs/train/'
                           'drone_bombard_train2/weights/best.pt'},
            {'inference_rate': 10.0}
        ]
    )

    xmarker_detector_delayed = TimerAction(
        period=8.0,
        actions=[xmarker_detector_node]
    )

    return LaunchDescription([
        # Declare arguments
        declare_world,
        declare_model,
        declare_x,
        declare_y,
        declare_z,
        declare_headless,
        declare_enable_path,
        declare_enable_vision,

        # Set environment
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        set_ld_library_path,
        set_ros_version,

        # Launch processes
        micro_xrce_agent,
        gazebo_server,
        gazebo_client,
        spawn_entity_delayed,
        px4_sitl_delayed,
        circle_path_delayed,
        xmarker_detector_delayed,
    ])
