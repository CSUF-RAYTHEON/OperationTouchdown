import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource # <--- Added AnyLaunch
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Path Setup
    pkg_description = get_package_share_directory('my_ugv_description')
    
    # 2. URDF/XACRO Processing
    xacro_file = os.path.join(pkg_description, 'urdf', 'ugv.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
   
    # 3. CHASSIS NODE
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 4. MOTOR NODE
    motor_driver = Node(
        package='my_ugv_hardware',
        executable='motor_driver',
        output='screen'
    )

    # 5. FOXGLOVE BRIDGE (The Fix is here!)
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([ # <--- Use AnyLaunchDescriptionSource for .xml files
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        ])
    )

    # 6. VIRTUAL ODOMETRY
    virtual_odom = Node(
        package='my_ugv_hardware',
        executable='virtual_odom',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        motor_driver,
        foxglove_bridge,
        virtual_odom,
    ])

