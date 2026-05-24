import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    # 1. Paths to external launch files
    foxglove_launch_path = os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
    rplidar_launch_path = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    oakd_launch_path = os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')

    # 2. Define the SLAM Lifecycle Node
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
                'base_frame': 'base_link', 
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_timeout': 1.0
            }
        ]
    )

    # 3. Automation Events (The "Auto-Key" for SLAM)
    # Trigger 'configure' immediately on start
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node is slam_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Trigger 'activate' once 'configure' is successful
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
        # 4. Foxglove Bridge
        IncludeLaunchDescription(AnyLaunchDescriptionSource(foxglove_launch_path)),

        # 5. Roboteq Motors (Fix: publish_tf=False to prevent conflicts)
        Node(
            package='roboteq_ros2_driver',
            executable='roboteq_bridge.py',
            name='roboteq_bridge',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'publish_tf': False, 
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }]
        ),

        # 6. Xbox Controller (joy_node)
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05}]
        ),

        # 7. Teleop Twist Joy (The Controller Mapping)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': 5,            # RB Button
                'axis_linear.x': 1,            # Left Stick Up/Down
                'axis_angular.yaw': 3,         # Right Stick Left/Right
                'scale_linear.x': 1.0,
                'scale_angular.yaw': 0.5,
            }]
        ),

        # 8. RPLidar A1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            launch_arguments={'serial_port': '/dev/ttyUSB0', 'serial_baudrate': '115200', 'frame_id': 'laser'}.items()
        ),

        # 9. OAK-D Camera (RGB enabled for AKD1000 YOLO; depth aligned to RGB by default).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oakd_launch_path),
            launch_arguments={
                'name': 'oak',
                'parent_frame': 'base_link',
                'enable_color': 'true',
                'enable_depth': 'true',
                'rectify_rgb': 'true',
                'pointcloud.enable': 'false',
            }.items()
        ),

        # 10. STATIC TRANSFORMS
        # NOTE: odom->base_link is intentionally removed — this must come from the EKF node.
        # A static odom->base_link would override EKF output and freeze the robot at the origin.
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser_tf', arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_oak_tf', arguments=['0.15', '0', '0.3', '0', '0', '0', 'base_link', 'oak']),

        # 11. DELAYED & AUTOMATED SLAM
        TimerAction(
            period=5.0,
            actions=[slam_node, configure_event, activate_event]
        )
    ])
