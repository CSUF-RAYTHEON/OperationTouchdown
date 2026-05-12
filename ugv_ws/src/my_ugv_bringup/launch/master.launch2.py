import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
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
    slam_params_path = os.path.join(pkg_share, 'config', 'slam_param.yaml')

    # 3. Core Infrastructure
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )

    foxglove_bridge = IncludeLaunchDescription(AnyLaunchDescriptionSource(foxglove_path))

    roboteq_bridge = Node(
        package='roboteq_ros2_driver', executable='roboteq_bridge.py', name='roboteq_bridge',
        parameters=[{'publish_tf': False, 'odom_frame': 'odom', 'base_frame': 'base_footprint'}]
    )

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[ekf_path]
    )

    # 4. Sensors
    rplidar_node = Node(
        package='rplidar_ros', executable='rplidar_node', name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'scan_frequency': 10.0,
        }]
    )

    oakd_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(oakd_path),
    launch_arguments={
        'name': 'oak',
        'parent_frame': 'base_link',
        'publish_tf_from_calibration': 'false', 
        'enable_depth': 'true',
        # --- NEW BUDGET CUTS ---
        'lr_check': 'true',             # Helps with depth accuracy
        'enable_rgb': 'false',          # SHUT OFF the color video (Massive CPU saver)
        'enable_pointcloud': 'false',   # SHUT OFF 3D points (The biggest hog)
        'enable_imu': 'true',           # Keep this for our EKF!
        'depth_fps': '10.0',            # Lower from 30fps to 10fps
    }.items()
)

    depth_to_scan = Node(
        package='depthimage_to_laserscan', executable='depthimage_to_laserscan_node',
        remappings=[('depth', '/oak/stereo/image_raw'), ('depth_camera_info', '/oak/stereo/camera_info'), ('scan', '/camera_scan')],
        parameters=[{'output_frame': 'oak_rgb_camera_frame', 'range_min': 0.45, 'range_max': 3.5}]
    )

    # 5. NEW: Dual Laser Merger Node
    # This node is the "Factory" that combines your Lidar and Camera data
    laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        namespace='', # Explicitly defined
        remappings=[
            ('/laser_1', '/scan'),
            ('/laser_2', '/camera_scan'),
            ('/merged', '/scan_merged')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'publish_rate': 100,
            'scan_time': 0.1,
        }]
    )

    joy_node = Node(package='joy_linux', executable='joy_linux_node', name='joy_node', parameters=[{'dev': '/dev/input/js0'}])
    
    teleop_node = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        parameters=[{'enable_button': 5, 'axis_linear.x': 3, 'axis_angular.yaw': 1, 'scale_linear.x': 0.5, 'scale_angular.yaw': 0.9}]
    )

    # 6. SLAM Configuration
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='', # REQUIRED: This fixes the TypeError
        output='screen',
        parameters=[slam_params_path]
    )

    # Lifecycle Management for SLAM
    configure_event = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam_toolbox), transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate_event = RegisterEventHandler(OnStateTransition(target_lifecycle_node=slam_toolbox, goal_state='inactive', entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam_toolbox), transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    # 7. Final Return with Staggered Timers
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
        # Start Merger after 5 seconds to ensure sensors are "Liquid"
        TimerAction(period=5.0, actions=[laser_merger_node]),
        # Start SLAM last, once the /scan_merged topic is fully established
        TimerAction(period=8.0, actions=[slam_toolbox, configure_event, activate_event])
    ])