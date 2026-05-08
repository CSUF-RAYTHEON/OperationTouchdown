"""
Akida-enabled camera launch for the UGV.

Wraps the upstream depthai_ros_driver camera launch and adds the
BrainChip Akida object-detection node from my_ugv_vision.

Usage
-----
# With Akida inference enabled (default):
  ros2 launch my_ugv_vision akida_camera.launch.py

# With a custom model path:
  ros2 launch my_ugv_vision akida_camera.launch.py \
    akida_model_path:=/path/to/model.fbz \
    akida_confidence_threshold:=0.25

# Camera only, no Akida (falls back to pure camera driver):
  ros2 launch my_ugv_vision akida_camera.launch.py use_akida:=false

How it works
------------
1.  Includes the unmodified upstream camera.launch.py from
    depthai_ros_driver so the OAK-D driver is started normally.
    camera.i_nn_type is forced to "none" via a parameter override
    so the OAK-D streams raw RGB only — no on-device MyriadX NN.

2.  When use_akida=true, a second Node (my_ugv_vision/object_detector)
    is started.  It subscribes to the raw RGB topic, runs each
    224×224 frame through ugv_object_detect_model.fbz on the Akida
    AKD1000 neuromorphic processor (PCIe/M.2), and publishes:
      ~/akida/detections        vision_msgs/Detection2DArray
      ~/akida/annotated_image   sensor_msgs/Image  (debug overlay)

Venv
----
akida, numpy, and opencv-python must be in the venv at:
  <ugv_ws>/akida_venv/
Run the setup script once to create it:
  bash ugv_ws/setup_akida_venv.sh

The node picks up the venv automatically via the AKIDA_VENV_SITE
environment variable injected by this launch file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Workspace-relative paths
# Resolve from this file's installed location back to ugv_ws/.
# This file installs to:
#   <ugv_ws>/install/my_ugv_vision/share/my_ugv_vision/launch/
# So ugv_ws/ is six levels up from __file__.
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# Walk up: launch/ → my_ugv_vision/share/ → my_ugv_vision/ → share/ → install/ → ugv_ws/
_UGV_WS = os.path.normpath(os.path.join(_THIS_DIR, *(['..'] * 5)))

# When running from source (colcon build --symlink-install or direct execution)
# the file may live inside src/ — detect and adjust.
if 'src' in _THIS_DIR:
    # src/my_ugv_vision/launch/ → src/ → ugv_ws/
    _UGV_WS = os.path.normpath(
        os.path.join(_THIS_DIR, '..', '..', '..')
    )

_AKIDA_VENV_SITE = os.path.join(
    _UGV_WS, 'akida_venv', 'lib', 'python3.12', 'site-packages'
)

_DEFAULT_FBZ = os.path.join(
    os.path.dirname(_UGV_WS), 'ugv_object_detect_model.fbz'
)


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_akida = LaunchConfiguration('use_akida').perform(context)
    akida_model_path = LaunchConfiguration('akida_model_path').perform(context)
    akida_image_topic = LaunchConfiguration('akida_image_topic').perform(context)
    akida_confidence = LaunchConfiguration('akida_confidence_threshold').perform(context)
    akida_bg_class = LaunchConfiguration('akida_background_class').perform(context)

    depthai_prefix = get_package_share_directory('depthai_ros_driver')
    my_vision_prefix = get_package_share_directory('my_ugv_vision')

    # ------------------------------------------------------------------
    # Camera driver — upstream, unmodified.
    # Force i_nn_type=none so the OAK-D sends raw RGB without running
    # any on-device NN when Akida is handling inference.
    # ------------------------------------------------------------------
    camera_extra_args = {}
    if use_akida == 'true':
        camera_extra_args['params_file'] = os.path.join(
            my_vision_prefix, 'config', 'akida_camera.yaml'
        )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'name': name,
            'namespace': namespace,
            'camera_model': LaunchConfiguration('camera_model'),
            'parent_frame': LaunchConfiguration('parent_frame'),
            'cam_pos_x': LaunchConfiguration('cam_pos_x'),
            'cam_pos_y': LaunchConfiguration('cam_pos_y'),
            'cam_pos_z': LaunchConfiguration('cam_pos_z'),
            'cam_roll': LaunchConfiguration('cam_roll'),
            'cam_pitch': LaunchConfiguration('cam_pitch'),
            'cam_yaw': LaunchConfiguration('cam_yaw'),
            'rectify_rgb': LaunchConfiguration('rectify_rgb'),
            'publish_tf_from_calibration': LaunchConfiguration(
                'publish_tf_from_calibration'
            ),
            **camera_extra_args,
        }.items(),
    )

    actions = [camera_launch]

    # ------------------------------------------------------------------
    # Akida inference node — only when use_akida=true
    # ------------------------------------------------------------------
    if use_akida == 'true':
        actions.append(
            Node(
                package='my_ugv_vision',
                executable='object_detector',
                name='akida_object_detector',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'model_path': akida_model_path,
                    'image_topic': akida_image_topic,
                    'confidence_threshold': float(akida_confidence),
                    'background_class': int(akida_bg_class),
                    'publish_annotated_image': True,
                }],
                additional_env={
                    # Provides akida, numpy, cv2 from the project venv
                    # without overriding system Python used by rclpy.
                    'AKIDA_VENV_SITE': _AKIDA_VENV_SITE,
                },
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Camera identity
        DeclareLaunchArgument('name', default_value='oak'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('camera_model', default_value='OAK-D-PRO'),
        DeclareLaunchArgument('parent_frame', default_value='oak-d-base-frame'),
        DeclareLaunchArgument('cam_pos_x', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_y', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_z', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
        DeclareLaunchArgument('rectify_rgb', default_value='true'),
        DeclareLaunchArgument('publish_tf_from_calibration', default_value='false'),

        # Akida inference
        DeclareLaunchArgument(
            'use_akida',
            default_value='true',
            description=(
                'Start the Akida BrainChip detector node. '
                'Automatically switches the camera to raw-RGB-only mode.'
            ),
        ),
        DeclareLaunchArgument(
            'akida_model_path',
            default_value=_DEFAULT_FBZ,
            description='Absolute path to ugv_object_detect_model.fbz.',
        ),
        DeclareLaunchArgument(
            'akida_image_topic',
            default_value='oak/rgb/image_raw',
            description='RGB image topic the Akida node subscribes to.',
        ),
        DeclareLaunchArgument(
            'akida_confidence_threshold',
            default_value='0.30',
            description='Min normalised confidence [0-1] to publish a detection.',
        ),
        DeclareLaunchArgument(
            'akida_background_class',
            default_value='-1',
            description=(
                'Class index treated as background (suppressed). '
                '-1 = publish all detections.'
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
