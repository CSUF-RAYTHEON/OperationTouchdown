#!/usr/bin/env python3
"""SLAM launch file — RPLidar + Roboteq + RSP + slam_toolbox + Foxglove."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── packages ────────────────────────────────────────────────────────────
    pkg_bringup     = get_package_share_directory('my_ugv_bringup')
    pkg_description = get_package_share_directory('my_ugv_description')
    pkg_slam        = get_package_share_directory('slam_toolbox')

    # ── arguments ───────────────────────────────────────────────────────────
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_bringup, 'config', 'slam_param.yaml'),
        description='Path to slam_toolbox YAML parameters file',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Launch Foxglove Bridge',
    )

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    use_foxglove     = LaunchConfiguration('use_foxglove')

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
            'use_sim_time': use_sim_time,
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
            'serial_port':    '/dev/ttyACM0',
            'use_sim_time':   use_sim_time,
        }],
    )

    # ── slam_toolbox async SLAM ──────────────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── LoRa bridge ──────────────────────────────────────────────────────────
    lora_bridge_node = Node(
        package='my_ugv_hardware',
        executable='lora_bridge',
        name='lora_bridge',
        output='screen',
        parameters=[{
            'lora_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'lora_baud': 9600,
        }],
    )

    # ── Foxglove Bridge ──────────────────────────────────────────────────────
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
        slam_params_file_arg,
        use_sim_time_arg,
        use_foxglove_arg,
        rsp_node,
        rplidar_node,
        motor_driver_node,
        slam_node,
        lora_bridge_node,
        foxglove_bridge,
    ])
