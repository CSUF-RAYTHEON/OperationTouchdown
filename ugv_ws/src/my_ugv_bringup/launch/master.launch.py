import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to other launch files

    foxglove_launch_path = os.path.join(
   	 get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')

    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')
    
    oakd_launch_path = os.path.join(
        get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')

    return LaunchDescription([
        # 2. Start Foxglove Bridge (Default port 8765)
        IncludeLaunchDescription(
   	 get_package_share_directory('foxglove_bridge') + '/launch/foxglove_bridge_launch.xml'
        ),

        # 3. Start Roboteq Hardware Bridge (Your custom script)
        Node(
            package='roboteq_ros2_driver',
            executable='roboteq_bridge.py',
            name='roboteq_bridge',
            parameters=[{'serial_port': '/dev/ttyAMA0'}] # Adjust port if needed
        ),

        # 4. Start RPLidar A1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            launch_arguments={'serial_port': '/dev/ttyUSB0'}.items() # Adjust port if needed
        ),

        # 5. Start OAK-D S2 Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oakd_launch_path),
            launch_arguments={
        'name': 'oak',
        'parent_frame': 'base_link',
        'cam_pos_x': '0.15',  # Offset from center of rover
        'cam_pos_z': '0.3',   # Height of camera
    }.items()
        ),

        # 6. Static TF: Rover to Lidar (Offsets in meters: x, y, z, yaw, pitch, roll)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x', '0.1', 
                '--y', '0', 
                '--z', '0.2', 
                '--yaw', '0', 
                '--pitch', '0', 
                '--roll', '0', 
                '--frame-id', 'base_link', 
                '--child-frame-id', 'laser'
            ]
        ),

        # 7. Static TF: Rover to OAK-D
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_oakd_tf',
            arguments=[
                '--x', '0.15', 
                '--y', '0', 
                '--z', '0.3', 
                '--yaw', '0', 
                '--pitch', '0', 
                '--roll', '0', 
                '--frame-id', 'base_link', 
                '--child-frame-id', 'oak'
            ]
    )
])

