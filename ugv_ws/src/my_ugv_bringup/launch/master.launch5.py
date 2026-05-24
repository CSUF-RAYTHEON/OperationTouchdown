"""master.launch5.py — full UGV stack including Akida AKD1000 perception.

Layers in (rough timing):
  t=0   foxglove_bridge, roboteq_bridge, joy, teleop
        OAK-D (color + depth aligned to RGB, no pointcloud),
        RPLidar A1, static TFs
  t=5   slam_toolbox lifecycle bring-up
  t=8   my_ugv_vision perception pipeline (akida_yolo + localizer +
        tracker + obstacle publisher)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    foxglove_launch_path = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch', 'foxglove_bridge_launch.xml')
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar_a1_launch.py')
    oakd_launch_path = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch', 'camera.launch.py')
    perception_launch_path = os.path.join(
        get_package_share_directory('my_ugv_vision'),
        'launch', 'perception.launch.py')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            '/opt/ros/jazzy/share/slam_toolbox/config/'
            'mapper_params_online_async.yaml',
            {
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_timeout': 1.0,
            },
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node is slam_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda node: node is slam_node,
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                ))
            ],
        ),
    )

    return LaunchDescription([
        IncludeLaunchDescription(AnyLaunchDescriptionSource(foxglove_launch_path)),

        Node(
            package='roboteq_ros2_driver',
            executable='roboteq_bridge.py',
            name='roboteq_bridge',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'publish_tf': False,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}],
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': 5,
                'axis_linear.x': 1,
                'axis_angular.yaw': 3,
                'scale_linear.x': 1.0,
                'scale_angular.yaw': 0.5,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '115200',
                'frame_id': 'laser',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oakd_launch_path),
            launch_arguments={
                'name': 'oak',
                'parent_frame': 'base_link',
                'enable_color': 'true',
                'enable_depth': 'true',
                'rectify_rgb': 'true',
                'pointcloud.enable': 'false',
            }.items(),
        ),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_laser_tf',
             arguments=['0.1', '0', '0.2', '0', '0', '0',
                        'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_oak_tf',
             arguments=['0.15', '0', '0.3', '0', '0', '0',
                        'base_link', 'oak']),

        TimerAction(
            period=5.0,
            actions=[slam_node, configure_event, activate_event],
        ),

        # Bring up perception last so OAK-D topics + TF + map frame exist.
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(perception_launch_path),
                ),
            ],
        ),
    ])
