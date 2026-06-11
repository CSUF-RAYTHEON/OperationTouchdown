"""Shared builder for the UGV master launch files.

This is the single source of truth for the full UGV stack so the four master
launch entry points stay thin and consistent:

    master.launch2.py        -> standalone controller stack (kept as-is, legacy)
    master_akida.launch.py        -> build_master(akida=True,  nav2=False)
    master_nav2.launch.py         -> build_master(akida=False, nav2=True)
    master_nav2_akida.launch.py   -> build_master(akida=True,  nav2=True)

``akida`` toggles the BrainChip AKD1000 perception node (object detection +
Foxglove text summary) and swaps the OAK camera config to one that publishes
the RGB stream the detector needs. ``nav2`` toggles the Nav2 autonomy stack.

It lives inside the importable ``my_ugv_bringup`` python package so the launch
files (which run from the installed ``share/`` dir) can ``import`` it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def build_master(akida: bool = False, nav2: bool = False) -> LaunchDescription:
    """Assemble the full UGV LaunchDescription.

    Args:
        akida: include the AKD1000 perception node and enable the OAK RGB stream.
        nav2: include the Nav2 autonomy stack (starts at t=60s after SLAM).
    """
    pkg_share = get_package_share_directory('my_ugv_bringup')

    # 1. Process URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_ugv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Paths
    foxglove_path = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch', 'foxglove_bridge_launch.xml')
    oakd_path = os.path.join(pkg_share, 'launch', 'oak_camera.launch.py')
    # Akida needs /oak/rgb/image_raw, so use the RGB-enabled camera config.
    oak_params_name = 'oak_camera_rgb.yaml' if akida else 'oak_camera.yaml'
    oak_params_path = os.path.join(pkg_share, 'config', oak_params_name)
    ekf_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_params_path = os.path.join(pkg_share, 'config', 'slam_param.yaml')

    # 3. Core Infrastructure
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 30.0,
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description_raw}],
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_path),
        launch_arguments={
            "topic_whitelist": "[\"/tf\", \"/tf_static\", \"/scan\", \"/odom\", \"/odometry/filtered\", \"/map\", \"/mission/state\", \"/local_costmap/costmap\", \"/global_costmap/costmap\", \"/akida/detected_objects\", \"/akida/inference_latency_ms\", \"/akida/image_annotated\", \"/diagnostics\", \"/lora/raw\", \"/robot_description\", \"/joint_states\", \"/clock\"]",
            "send_buffer_limit": "10000000",
        }.items())

    roboteq_bridge = Node(
        package='roboteq_ros2_driver', executable='roboteq_bridge.py',
        name='roboteq_bridge',
        parameters=[{
            'publish_tf': False,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'drive_invert_linear': False,
            'odom_invert_linear': 1.0,
        }],
        remappings=[("cmd_vel", "/cmd_vel_motor")],
    )

    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_path],
        arguments=['--ros-args', '--log-level', 'rclcpp:=error'],
    )

    # 4. Sensors
    # Use the stable by-id symlink: the RPLidar A1's interface board is a
    # Silicon Labs CP210x. Addressing it by /dev/ttyUSB* is fragile because USB
    # enumeration order swaps it with the CH340-based LoRa module (which then
    # makes rplidar_node time out on the wrong device and die — no scan/no spin).
    rplidar_node = Node(
        package='rplidar_ros', executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'frame_id': 'laser',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'scan_frequency': 5.0,
            'inverted': False,
            'min_dist': 0.4,
        }]
    )

    oakd_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_path),
        launch_arguments={
            'name': 'oak',
            'parent_frame': 'camera_link',
            'params_file': oak_params_path,
            'cam_pos_x': '0.0',
            'cam_pos_y': '0.0',
            'cam_pos_z': '0.0',
            'cam_roll': '0.0',
            'cam_pitch': '0.0',
            'cam_yaw': '0.0',
            'publish_tf_from_calibration': 'true',
        }.items(),
    )

    camera_link_to_oak_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_oak_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'oak'],
    )

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_link', 'camera_depth_frame'],
    )

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        remappings=[('depth', '/oak/stereo/image_raw'),
                    ('depth_camera_info', '/oak/stereo/camera_info'),
                    ('scan', '/camera_scan')],
        parameters=[{
            'range_min': 0.35,
            'range_max': 3.5,
            'output_frame': 'camera_link',
            'scan_height': 3,
            'qos_overrides./depth.subscription.reliability': 'reliable',
            'qos_overrides./depth_camera_info.subscription.reliability': 'reliable',
        }]
    )

    laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        parameters=[{
            'laser_1_topic': '/scan',
            'laser_2_topic': '/camera_scan',
            'merged_scan_topic': '/scan_merged',
            'target_frame': 'base_footprint',
            'tolerance': 0.1,
            'queue_size': 10,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 12.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'enable_shadow_filter': False,
            'enable_average_filter': False,
            'enable_calibration': False,
        }]
    )

    joy_node = Node(
        package='joy_linux', executable='joy_linux_node', name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.02}])

    teleop_node = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'enable_button': 7,
            'axis_linear': {'x': 1},
            'axis_angular': {'yaw': 0},
            'scale_linear': {'x': 0.15},
            'scale_angular': {'yaw': 0.9},
        }]
    )

    joy_speed_estop = Node(
        package="my_ugv_bringup",
        executable="joy_speed_estop.py",
        name="joy_speed_estop",
        output="screen",
    )

    # 6. SLAM — delegate lifecycle to slam_toolbox's online_async_launch.py.
    slam_launch = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'slam_params_file': slam_params_path,
                    'use_sim_time': 'false',
                }.items(),
            )
        ],
    )

    slam_activator = TimerAction(
        period=35.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                    'source /opt/ros/jazzy/setup.bash && '
                    'source /home/ugv/Desktop/OperationTouchdown/ugv_ws/install/setup.bash && '
                    'for i in 1 2 3 4 5 6 7 8; do '
                    '  STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null); '
                    '  echo "[slam_activator] attempt $i: $STATE"; '
                    '  case "$STATE" in '
                    '    *active*) echo "[slam_activator] SLAM is active."; exit 0;; '
                    '    *inactive*) '
                    '      ros2 lifecycle set /slam_toolbox activate 2>&1; '
                    '      sleep 3;; '
                    '    *) echo "[slam_activator] SLAM not found, waiting..."; sleep 5;; '
                    '  esac; '
                    'done; '
                    'echo "[slam_activator] WARNING: gave up after 8 attempts — check SLAM logs"'
                ],
                output='screen',
                shell=False,
            )
        ]
    )

    # 7. Competition nodes
    lora_bridge = Node(
        package='my_ugv_hardware',
        executable='lora_bridge',
        name='lora_bridge',
        output='screen',
        # CH340-based LoRa module, addressed by stable by-id symlink (see rplidar note).
        parameters=[{'lora_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 'lora_baud': 9600}],
    )

    mission_controller = Node(
        package='my_ugv_hardware',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{'challenge_mode': LaunchConfiguration('challenge_mode')}],
    )

    kill_switch = Node(
        package='my_ugv_hardware',
        executable='kill_switch',
        name='kill_switch',
        output='screen',
    )

    # 8. Akida perception (object detection + Foxglove text summary).
    # Delayed so OAK RGB topics exist before the detector subscribes.
    akida_yolo_node = Node(
        package='my_ugv_vision',
        executable='akida_yolo_node',
        name='akida_yolo_node',
        output='screen',
        parameters=[{
            'image_topic': '/oak/rgb/image_raw',
            'detections_topic': '/akida/detections',
            'annotated_topic': '/akida/image_annotated',
            'latency_topic': '/akida/inference_latency_ms',
            'objects_topic': '/akida/detected_objects',
            'score_threshold': 0.35,
            'iou_threshold': 0.45,
            'publish_annotated': True,
        }],
    )

    enable_laser_merger = LaunchConfiguration('enable_laser_merger')

    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_composition': 'False',
        }.items(),
    )

    # 9. Assemble
    actions = [
        DeclareLaunchArgument(
            'challenge_mode',
            default_value='2',
            description='Competition challenge mode: 1 (straight), 2 (Nav2 goal), or 3 (Nav2 goal + obstacles)',
        ),
        DeclareLaunchArgument(
            'enable_laser_merger',
            default_value='false',
            description=(
                'Fuse /scan + /camera_scan -> /scan_merged. '
                'Outdoor (default): false. Indoor: true for close-range obstacles.'
            ),
        ),
        joint_state_publisher,
        robot_state_publisher,
        foxglove_bridge,
        roboteq_bridge,
        ekf_node,
        rplidar_node,
        oakd_camera,
        camera_link_to_oak_tf,
        camera_depth_tf,
        depth_to_scan,
        joy_node,
        teleop_node,
        joy_speed_estop,
        lora_bridge,
        mission_controller,
        kill_switch,
        TimerAction(
            period=42.0,
            actions=[laser_merger_node],
            condition=IfCondition(enable_laser_merger),
        ),
        slam_launch,
        slam_activator,
    ]

    if akida:
        actions.append(TimerAction(period=8.0, actions=[akida_yolo_node]))

    if nav2:
        # Nav2 starts at t=60s to guarantee map->odom TF from SLAM is live.
        actions.append(TimerAction(period=60.0, actions=[navigation]))

    return LaunchDescription(actions)
