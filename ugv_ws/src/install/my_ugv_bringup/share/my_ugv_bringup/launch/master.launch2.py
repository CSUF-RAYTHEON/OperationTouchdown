import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg
import xacro

def generate_launch_description():
    # 1. Path Setup
    pkg_share = get_package_share_directory('my_ugv_bringup')
    
    # Process URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_ugv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Paths
    foxglove_path = os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    oakd_launch_path = os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 2. SLAM Lifecycle Node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='', 
        output='screen',
        parameters=[
            '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml',
            {
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'scan_topic': '/scan',
                'mode': 'mapping'
            }
        ]
    )

    # 3. SLAM Activation Logic
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node is slam_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
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
            ]
        )
    )

    return LaunchDescription([
        # 4. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description_raw}]
        ),

        # 5. Foxglove Bridge (Using AnyLaunchDescriptionSource for XML compatibility)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(foxglove_path)
        ),

        # 6. Roboteq Motors
        Node(
            package='roboteq_ros2_driver',
            executable='roboteq_bridge.py',
            name='roboteq_bridge',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'publish_tf': False,
                'odom_frame': 'odom',
                'base_frame': 'base_footprint'
            }]
        ),

        # 7. EKF Sensor Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 8. Controller Support
        Node(package='joy_linux', executable='joy_linux_node', name='joy_node', parameters=[{'dev': '/dev/input/js0'}]),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[{'enable_button': 5, 'axis_linear.x': 1, 'axis_angular.yaw': 3, 'scale_linear.x': 3.0, 'scale_angular.yaw': 1.0}]
        ),

        # 9. Hardware Drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '115200',
                'frame_id': 'laser'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oakd_launch_path),
            launch_arguments={'name': 'oak', 'enable_rgb': 'false'}.items()
        ),

        # 10. Start SLAM after a short delay
        TimerAction(
            period=5.0,
            actions=[slam_node, configure_event, activate_event]
        )
    ])
