#!/usr/bin/env python3
"""Master launch — selects operating mode: slam | nav | manual.

Usage:
  ros2 launch my_ugv_bringup master.launch.py mode:=slam
  ros2 launch my_ugv_bringup master.launch.py mode:=nav map:=/path/to/map.yaml
  ros2 launch my_ugv_bringup master.launch.py mode:=manual
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    NotEqualsSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup     = get_package_share_directory('my_ugv_bringup')
    pkg_description = get_package_share_directory('my_ugv_description')

    # ── arguments ───────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Operating mode: slam | nav | manual',
        choices=['slam', 'nav', 'manual'],
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Map YAML path (required for nav mode)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )
    challenge_mode_arg = DeclareLaunchArgument(
        'challenge_mode',
        default_value='2',
        description='Competition challenge: 1, 2, or 3',
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Launch Foxglove Bridge',
    )

    mode           = LaunchConfiguration('mode')
    map_file       = LaunchConfiguration('map')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    challenge_mode = LaunchConfiguration('challenge_mode')
    use_foxglove   = LaunchConfiguration('use_foxglove')

    # ── URDF ─────────────────────────────────────────────────────────────────
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

    # ── Foxglove Bridge (always launched when enabled) ───────────────────────
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

    # ── SLAM mode ─────────────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_foxglove': 'false',   # foxglove already launched above
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
    )

    # ── NAV mode ──────────────────────────────────────────────────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map':            map_file,
            'use_sim_time':   use_sim_time,
            'challenge_mode': challenge_mode,
            'use_foxglove':   'false',
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
    )

    # ── MANUAL mode ───────────────────────────────────────────────────────────
    motor_driver_manual = Node(
        package='my_ugv_hardware',
        executable='motor_driver',
        name='roboteq_bridge',
        output='screen',
        parameters=[{
            'serial_port':  '/dev/ttyACM0',
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'manual'"])),
    )

    virtual_odom_manual = Node(
        package='my_ugv_hardware',
        executable='virtual_odom',
        name='virtual_odom',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'manual'"])),
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        use_sim_time_arg,
        challenge_mode_arg,
        use_foxglove_arg,
        rsp_node,
        foxglove_bridge,
        slam_launch,
        nav_launch,
        motor_driver_manual,
        virtual_odom_manual,
    ])
