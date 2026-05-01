import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessExit # Correct Core Library
import lifecycle_msgs.msg
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('my_ugv_bringup')
    
    # 1. Process URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_ugv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Paths
    foxglove_path = os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
    oakd_path = os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
    ekf_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 3. Define Departments (Nodes)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )

    foxglove_bridge = IncludeLaunchDescription(AnyLaunchDescriptionSource(foxglove_path))

    roboteq_bridge = Node(
        package='roboteq_ros2_driver',
        executable='roboteq_bridge.py',
        name='roboteq_bridge',
        parameters=[{'publish_tf': False, 'odom_frame': 'odom', 'base_frame': 'base_footprint'}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_path]
    )

    # 4. Lidar with Direct Node Oversight (Required for OnProcessExit)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'frame_id': 'laser',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
	    'angle_compensate': True,
        }],
        output='screen'
    )

    rplidar_respawn_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rplidar_node,
            on_exit=[
                LogInfo(msg='Lidar Market Crash! Restarting in 1 second...'),
                TimerAction(period=1.0, actions=[rplidar_node])
            ]
        )
    )

    oakd_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_path),
        launch_arguments={'name': 'oak', 'enable_rgb': 'false'}.items()
    )

    # 5. Controllers
    joy_node = Node(package='joy_linux', executable='joy_linux_node', name='joy_node', parameters=[{'dev': '/dev/input/js0'}])
    
    teleop_node = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        parameters=[{'enable_button': 5, 'axis_linear.x': 1, 'axis_angular.yaw': 3, 'scale_linear.x': 1.0, 'scale_angular.yaw': 0.5}]
    )

    # 6. SLAM (The Lifecycle Department)
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='', # REQUIRED: Fixed Zoning Violation
        output='screen',
        parameters=[
            '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml',
            {'use_sim_time': False, 'odom_frame': 'odom', 'base_frame': 'base_footprint', 'scan_topic': '/scan', 'mode': 'mapping'}
        ]
    )

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda node: node is slam_toolbox,
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox,
        start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda node: node is slam_toolbox,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ))]
    ))

    # 7. Final Return Description
    return LaunchDescription([
        robot_state_publisher,
        foxglove_bridge,
        roboteq_bridge,
        ekf_node,
        rplidar_respawn_handler,
        rplidar_node, # We start the node, and the handler watches it
        oakd_camera,
        joy_node,
        teleop_node,
        TimerAction(period=5.0, actions=[slam_toolbox, configure_event, activate_event])
    ])
