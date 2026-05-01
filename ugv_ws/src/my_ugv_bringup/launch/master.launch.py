import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessExit 
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
    # Point to your custom SLAM policy (ensuring name matches slam_param.yaml)
    slam_params_path = os.path.join(pkg_share, 'config', 'slam_param.yaml')

    # 3. Core Infrastructure Nodes
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
        parameters=[{
            'publish_tf': False, 
            'odom_frame': 'odom', 
            'base_frame': 'base_footprint',
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_path]
    )

    # 4. Lidar (Downsampled to 7Hz for Efficiency)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
            'scan_frequency': 5.0,
        }],
        output='screen'
    )

    oakd_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_path),
        launch_arguments={
            'name': 'oak',
            'enable_color': 'false',       # STOPS the RGB stream
            'rectify_rgb': 'false',        # SAVES massive CPU math
            'enable_depth': 'true',        # KEEPS the data needed for SLAM
            'depth_module.depth_profile': '640,400,5', # 5Hz is plenty for safety
        }.items()
    )

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/oak/stereo/image_raw'),       
            ('depth_camera_info', '/oak/stereo/camera_info'), 
            ('scan', '/camera_scan')
        ],
        parameters=[{
            'output_frame': 'oak_rgb_camera_frame', 
            'range_min': 0.45,  # Increased min range to skip noisy near-pixels
            'range_max': 3.5,   # LOWERED from 5.0 to 3.5 (Saves massive math)
            'scan_height': 1,
            'scan_time': 0.2    # MATCHED to 5Hz (0.2s) to sync with OAK-D FPS
        }]
    )

    # 6. User Interface Nodes
    joy_node = Node(package='joy_linux', executable='joy_linux_node', name='joy_node', parameters=[{'dev': '/dev/input/js0'}])
    
    teleop_node = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        parameters=[{'enable_button': 5, 'axis_linear.x': 1, 'axis_angular.yaw': 3, 'scale_linear.x': 3.0, 'scale_angular.yaw': 1.0}]
    )

    # 7. SLAM (Restructured for "Market Activation")
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_path]
    )

    # FIXED: Using matches_action instead of lambda for better "Traceability"
    from launch.events import matches_action

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            goal_state='inactive', # When it finishes configuring
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    # 8. Final Return
    return LaunchDescription([
        robot_state_publisher,
        foxglove_bridge,
        roboteq_bridge,
        ekf_node,
        rplidar_node,
        oakd_camera,
        depth_to_scan, 
        joy_node,
        teleop_node,
        # SLAM is started with a slight delay to let the Lidar and Odom "Solvency" stabilize
        TimerAction(period=3.0, actions=[slam_toolbox, configure_event, activate_event])
    ])