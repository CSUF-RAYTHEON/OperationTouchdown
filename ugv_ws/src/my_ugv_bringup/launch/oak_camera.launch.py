"""OAK-D driver only — no extra robot_state_publisher (avoids split TF tree)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('my_ugv_bringup')
    name = LaunchConfiguration('name').perform(context)
    parent_frame = LaunchConfiguration('parent_frame').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    publish_tf = LaunchConfiguration('publish_tf_from_calibration').perform(context) == 'true'

    tf_params = {}
    if publish_tf:
        tf_params = {
            'camera': {
                'i_publish_tf_from_calibration': True,
                'i_tf_tf_prefix': name,
                'i_tf_base_frame': name,
                'i_tf_parent_frame': parent_frame,
                'i_tf_cam_pos_x': LaunchConfiguration('cam_pos_x').perform(context),
                'i_tf_cam_pos_y': LaunchConfiguration('cam_pos_y').perform(context),
                'i_tf_cam_pos_z': LaunchConfiguration('cam_pos_z').perform(context),
                'i_tf_cam_roll': LaunchConfiguration('cam_roll').perform(context),
                'i_tf_cam_pitch': LaunchConfiguration('cam_pitch').perform(context),
                'i_tf_cam_yaw': LaunchConfiguration('cam_yaw').perform(context),
                'i_tf_imu_from_descr': 'true',
            }
        }

    if not params_file:
        params_file = os.path.join(pkg_share, 'config', 'oak_camera.yaml')

    log_level = 'debug' if os.environ.get('DEPTHAI_DEBUG') == '1' else 'info'

    return [
        ComposableNodeContainer(
            name=f'{name}_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name=name,
                    parameters=[ParameterFile(params_file, allow_substs=True), tf_params],
                )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('my_ugv_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='oak'),
        DeclareLaunchArgument('parent_frame', default_value='camera_link'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'oak_camera.yaml'),
        ),
        DeclareLaunchArgument('cam_pos_x', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_y', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_z', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_tf_from_calibration', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
