"""Lift 2D YOLO detections into 3D ``map``-frame poses.

Fuses OAK-D depth (5x5 median at bbox center, requires depth aligned to RGB)
with a LiDAR range sampled at the detection bearing. LiDAR is preferred when
its range is below ``lidar_max_trust_m`` (default 4.5 m for an outdoor RPLidar A1).

Sub:
  /akida/detections    vision_msgs/Detection2DArray
  /oak/stereo/image_raw sensor_msgs/Image (16UC1, mm)
  /oak/rgb/camera_info sensor_msgs/CameraInfo
  /scan_merged         sensor_msgs/LaserScan (cached)
Pub:
  /akida/landmarks_raw vision_msgs/Detection3DArray
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, LookupException, TransformListener
import tf2_geometry_msgs  # noqa: F401  registers PointStamped transform
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from my_ugv_vision.geometry_utils import (
    bearing_from_pixel,
    median_depth,
    min_range_in_window,
    pixel_to_3d,
)

DEFAULT_CLASS_SIZES: Dict[str, Tuple[float, float, float]] = {
    'traffic_cones': (0.30, 0.30, 0.50),
    'home-depot-bucket': (0.30, 0.30, 0.40),
    'Box': (0.40, 0.40, 0.40),
}


class ObjectLocalizerNode(Node):

    def __init__(self) -> None:
        super().__init__('object_localizer_node')

        self.declare_parameter('detections_topic', '/akida/detections')
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('rgb_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('scan_topic', '/scan_merged')
        self.declare_parameter('landmarks_topic', '/akida/landmarks_raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter(
            'rgb_optical_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('depth_window_px', 5,
                               ParameterDescriptor(
                                   type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('depth_min_m', 0.30)
        self.declare_parameter('depth_max_m', 8.0)
        self.declare_parameter('lidar_window_rad', 0.05)
        self.declare_parameter('lidar_max_trust_m', 4.5)
        self.declare_parameter('lidar_max_m', 12.0)
        self.declare_parameter('sync_slop_s', 0.1)
        self.declare_parameter('transform_timeout_s', 0.2)
        self.declare_parameter('min_detection_score', 0.35)

        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self._cam_ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._latest_scan: Optional[LaserScan] = None
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data)

        from vision_msgs.msg import Detection2DArray
        sub_det = Subscriber(self, Detection2DArray,
                             str(self.get_parameter('detections_topic').value))
        sub_depth = Subscriber(self, Image,
                               str(self.get_parameter('depth_topic').value),
                               qos_profile=qos_profile_sensor_data)
        sub_info = Subscriber(self, CameraInfo,
                              str(self.get_parameter('rgb_info_topic').value))
        self._sync = ApproximateTimeSynchronizer(
            [sub_det, sub_depth, sub_info],
            queue_size=10,
            slop=float(self.get_parameter('sync_slop_s').value))
        self._sync.registerCallback(self._on_synced)

        self.pub_landmarks = self.create_publisher(
            Detection3DArray,
            str(self.get_parameter('landmarks_topic').value), 10)

        self.get_logger().info(
            "object_localizer_node ready (LiDAR-preferred within "
            f"{float(self.get_parameter('lidar_max_trust_m').value)} m).")

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _ensure_cam(self, info: CameraInfo) -> None:
        if self._cam_ready:
            return
        self.cam_model.fromCameraInfo(info)
        self._cam_ready = True
        self.get_logger().info(
            f"camera intrinsics ready: fx={self.cam_model.fx():.1f} "
            f"fy={self.cam_model.fy():.1f} cx={self.cam_model.cx():.1f} "
            f"cy={self.cam_model.cy():.1f} ({info.width}x{info.height})")

    def _on_synced(self, det_msg, depth_msg: Image, info_msg: CameraInfo) -> None:
        self._ensure_cam(info_msg)
        if not self._cam_ready:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as exc:
            self.get_logger().error(f"cv_bridge depth convert failed: {exc}")
            return

        # Depth must be aligned to RGB so we can sample at the same (cx, cy)
        # as the detection. If sizes differ from the RGB intrinsics by more
        # than 4 px, the i_align_depth pipeline is mis-configured.
        if (abs(depth.shape[1] - info_msg.width) > 4
                or abs(depth.shape[0] - info_msg.height) > 4):
            self.get_logger().error(
                f"depth shape {depth.shape[1]}x{depth.shape[0]} != RGB info "
                f"{info_msg.width}x{info_msg.height}; set "
                "stereo.i_align_depth:=true in depthai launch.")
            return

        out = Detection3DArray()
        out.header.stamp = det_msg.header.stamp
        out.header.frame_id = str(self.get_parameter('map_frame').value)

        depth_win = int(self.get_parameter('depth_window_px').value)
        depth_min = float(self.get_parameter('depth_min_m').value)
        depth_max = float(self.get_parameter('depth_max_m').value)
        lidar_win = float(self.get_parameter('lidar_window_rad').value)
        lidar_trust = float(self.get_parameter('lidar_max_trust_m').value)
        lidar_max = float(self.get_parameter('lidar_max_m').value)
        min_score = float(self.get_parameter('min_detection_score').value)
        tf_timeout = float(self.get_parameter('transform_timeout_s').value)
        optical_frame = str(self.get_parameter('rgb_optical_frame').value)

        for det in det_msg.detections:
            label = det.id if det.id else ''
            score = 0.0
            if det.results:
                first = det.results[0]
                inner = getattr(first, 'hypothesis', None)
                if inner is not None:
                    if not label:
                        label = inner.class_id
                    score = float(inner.score)
            if score < min_score:
                continue

            cx_px = float(det.bbox.center.position.x)
            cy_px = float(det.bbox.center.position.y)

            z_oak = median_depth(depth, cx_px, cy_px, depth_win,
                                 depth_min, depth_max)

            theta = bearing_from_pixel(cx_px, self.cam_model.fx(),
                                       self.cam_model.cx())
            z_lidar = float('nan')
            if self._latest_scan is not None:
                scan_frame = self._latest_scan.header.frame_id
                # Get the camera ray direction in the scan frame. Transform
                # two points (origin + tip) so translation cancels and we
                # recover pure rotation. Optical-frame convention: +z forward,
                # +x right, +y down. Robot/laser frame: +x forward, +y left.
                try:
                    p0 = PointStamped()
                    p0.header.frame_id = optical_frame
                    p0.header.stamp = det_msg.header.stamp
                    p0.point.x = 0.0
                    p0.point.y = 0.0
                    p0.point.z = 0.0
                    p1 = PointStamped()
                    p1.header = p0.header
                    p1.point.x = math.sin(theta)
                    p1.point.y = 0.0
                    p1.point.z = math.cos(theta)
                    dur = rclpy.duration.Duration(seconds=tf_timeout)
                    p0_s = self.tf_buffer.transform(p0, scan_frame, timeout=dur)
                    p1_s = self.tf_buffer.transform(p1, scan_frame, timeout=dur)
                    dx = p1_s.point.x - p0_s.point.x
                    dy = p1_s.point.y - p0_s.point.y
                    theta_scan = math.atan2(dy, dx)
                except Exception as exc:
                    self.get_logger().debug(
                        f"scan-frame bearing transform failed: {exc}; "
                        "using raw theta")
                    theta_scan = theta
                z_lidar_candidate = min_range_in_window(
                    self._latest_scan, theta_scan, lidar_win)
                if math.isfinite(z_lidar_candidate) and z_lidar_candidate <= lidar_max:
                    z_lidar = z_lidar_candidate

            if math.isfinite(z_lidar) and z_lidar <= lidar_trust:
                z = z_lidar
                src = 'lidar'
            elif math.isfinite(z_oak):
                z = z_oak
                src = 'oak'
            elif math.isfinite(z_lidar):
                z = z_lidar
                src = 'lidar_far'
            else:
                continue

            x_cam, y_cam, z_cam = pixel_to_3d(self.cam_model, cx_px, cy_px, z)
            if not all(map(math.isfinite, (x_cam, y_cam, z_cam))):
                continue

            p = PointStamped()
            p.header.frame_id = optical_frame
            p.header.stamp = det_msg.header.stamp
            p.point.x = float(x_cam)
            p.point.y = float(y_cam)
            p.point.z = float(z_cam)
            try:
                p_map = self.tf_buffer.transform(
                    p, str(self.get_parameter('map_frame').value),
                    timeout=rclpy.duration.Duration(seconds=tf_timeout))
            except (LookupException, Exception) as exc:
                self.get_logger().debug(
                    f"tf to map failed (label={label}): {exc}")
                continue

            d3 = Detection3D()
            d3.header = out.header
            d3.id = label
            hyp = ObjectHypothesisWithPose()
            inner = ObjectHypothesis()
            inner.class_id = label
            inner.score = float(score)
            hyp.hypothesis = inner
            hyp.pose.pose.position.x = p_map.point.x
            hyp.pose.pose.position.y = p_map.point.y
            hyp.pose.pose.position.z = p_map.point.z
            hyp.pose.pose.orientation.w = 1.0
            d3.results.append(hyp)

            bbox3 = BoundingBox3D()
            bbox3.center.position.x = p_map.point.x
            bbox3.center.position.y = p_map.point.y
            bbox3.center.position.z = p_map.point.z
            bbox3.center.orientation.w = 1.0
            size = DEFAULT_CLASS_SIZES.get(label, (0.3, 0.3, 0.3))
            bbox3.size.x = float(size[0])
            bbox3.size.y = float(size[1])
            bbox3.size.z = float(size[2])
            d3.bbox = bbox3

            out.detections.append(d3)

            self.get_logger().debug(
                f"localized {label} via {src} z={z:.2f}m -> "
                f"map=({p_map.point.x:.2f},{p_map.point.y:.2f})")

        self.pub_landmarks.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
