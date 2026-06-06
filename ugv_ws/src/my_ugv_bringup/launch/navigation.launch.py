#!/usr/bin/env python3
"""Full navigation launch — RPLidar + Roboteq + RSP + Nav2 + LoRa + Mission + Kill + Foxglove."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── packages ────────────────────────────────────────────────────────────
    pkg_bringup     = get_package_share_directory('my_ugv_bringup')
    pkg_description = get_package_share_directory('my_ugv_description')
    pkg_nav2        = get_package_share_directory('nav2_bringup')

    # ── arguments ───────────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to saved map YAML (required for localization mode)',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 parameters YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )
    challenge_mode_arg = DeclareLaunchArgument(
        'challenge_mode',
        default_value='2',
        description='Competition challenge mode: 1, 2, or 3',
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Launch Foxglove Bridge',
    )

    map_file       = LaunchConfiguration('map')
    params_file    = LaunchConfiguration('params_file')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    challenge_mode = LaunchConfiguration('challenge_mode')
    use_foxglove   = LaunchConfiguration('use_foxglove')

    # ── URDF / Robot State Publisher ────────────────────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf', 'ugv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time':      use_sim_time,
        }],
    )

    # ── RPLidar A1 ──────────────────────────────────────────────────────────
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    # ── Roboteq motor driver ─────────────────────────────────────────────────
    motor_driver_node = Node(
        package='my_ugv_hardware',
        executable='motor_driver',
        name='roboteq_bridge',
        output='screen',
        parameters=[{
            'serial_port':  '/dev/ttyACM0',
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Nav2 bringup (localization + navigation) ─────────────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':           map_file,
            'use_sim_time':  use_sim_time,
            'params_file':   params_file,
            'slam':          'False',
            'autostart':     'True',
        }.items(),
    )

    # ── LoRa bridge ──────────────────────────────────────────────────────────
    lora_bridge_node = Node(
        package='my_ugv_hardware',
        executable='lora_bridge',
        name='lora_bridge',
        output='screen',
        parameters=[{
            'lora_port': '/dev/ttyUSB2',
            'lora_baud': 9600,
        }],
    )

    # ── Mission controller ────────────────────────────────────────────────────
    mission_node = Node(
        package='my_ugv_hardware',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'challenge_mode':   challenge_mode,
            'straight_speed':   0.1,
            'goal_tolerance':   1.52,
            'mission_log_path': '/home/ugv/mission_log.txt',
            'use_sim_time':     use_sim_time,
        }],
    )

    # ── Kill switch ───────────────────────────────────────────────────────────
    kill_switch_node = Node(
        package='my_ugv_hardware',
        executable='kill_switch',
        name='kill_switch',
        output='screen',
        parameters=[{
            'gpio_pin':     17,
            'use_gpio':     True,
            'active_low':   True,
            'poll_rate_hz': 20.0,
        }],
    )

    # ── Foxglove Bridge ───────────────────────────────────────────────────────
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            )
        ),
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        challenge_mode_arg,
        use_foxglove_arg,
        rsp_node,
        rplidar_node,
        motor_driver_node,
        nav2_bringup,
        lora_bridge_node,
        mission_node,
        kill_switch_node,
        foxglove_bridge,
    ])
