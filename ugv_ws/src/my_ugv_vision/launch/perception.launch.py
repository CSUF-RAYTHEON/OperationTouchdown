import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_ugv_vision')
    default_model = os.path.join(pkg_share, 'models', 'ugv_yolo_akida.fbz')
    default_meta = os.path.join(pkg_share, 'models', 'model_metadata.json')

    akida_yaml = os.path.join(pkg_share, 'config', 'akida_yolo.yaml')
    localizer_yaml = os.path.join(pkg_share, 'config', 'object_localizer.yaml')
    tracker_yaml = os.path.join(pkg_share, 'config', 'landmark_tracker.yaml')
    obstacle_yaml = os.path.join(pkg_share, 'config', 'obstacle_publisher.yaml')

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value=default_model,
        description='Absolute path to the .fbz Akida model')
    metadata_arg = DeclareLaunchArgument(
        'metadata_path', default_value=default_meta,
        description='Absolute path to model_metadata.json')

    model_path = LaunchConfiguration('model_path')
    metadata_path = LaunchConfiguration('metadata_path')

    akida_yolo = Node(
        package='my_ugv_vision',
        executable='akida_yolo_node',
        name='akida_yolo_node',
        output='screen',
        parameters=[akida_yaml,
                    {'model_path': model_path,
                     'metadata_path': metadata_path}],
    )

    localizer = Node(
        package='my_ugv_vision',
        executable='object_localizer_node',
        name='object_localizer_node',
        output='screen',
        parameters=[localizer_yaml],
    )

    tracker = Node(
        package='my_ugv_vision',
        executable='landmark_tracker_node',
        name='landmark_tracker_node',
        output='screen',
        parameters=[tracker_yaml],
    )

    obstacles = Node(
        package='my_ugv_vision',
        executable='obstacle_publisher_node',
        name='obstacle_publisher_node',
        output='screen',
        parameters=[obstacle_yaml],
    )

    return LaunchDescription([
        model_path_arg,
        metadata_arg,
        akida_yolo,
        localizer,
        tracker,
        obstacles,
    ])
