"""Publish tracked landmarks as a PointCloud2 for Nav2's ObstacleLayer.

For each landmark, emits N points on a circle at ground height (z = 0) at the
class-specific radius, plus one center point at half class height. Frame =
``map``. Topic = ``/akida/obstacle_points``. Rate = 5 Hz default.
"""

from __future__ import annotations

import math
import struct
from typing import Dict, List, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

CLASS_RADII: Dict[str, Tuple[float, float]] = {
    'traffic_cones': (0.15, 0.50),
    'home-depot-bucket': (0.18, 0.40),
    'Box': (0.25, 0.40),
}
DEFAULT_RADIUS = (0.20, 0.40)


class ObstaclePublisherNode(Node):

    def __init__(self) -> None:
        super().__init__('obstacle_publisher_node')

        self.declare_parameter('input_topic', '/akida/landmark_array')
        self.declare_parameter('output_topic', '/akida/obstacle_points')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('obstacle_hz', 5.0)
        self.declare_parameter('ring_points', 12,
                               ParameterDescriptor(
                                   type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('intensity', 100.0)

        self._latest: Detection3DArray = Detection3DArray()
        self.create_subscription(
            Detection3DArray,
            str(self.get_parameter('input_topic').value),
            self._on_landmarks, 10)
        self.pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('output_topic').value), 10)

        rate = float(self.get_parameter('obstacle_hz').value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)

        self.get_logger().info(
            f"obstacle_publisher_node ready ({rate} Hz, ring="
            f"{int(self.get_parameter('ring_points').value)})")

    def _on_landmarks(self, msg: Detection3DArray) -> None:
        self._latest = msg

    def _tick(self) -> None:
        msg = self._latest
        if msg is None:
            return

        n_ring = max(0, int(self.get_parameter('ring_points').value))
        intensity = float(self.get_parameter('intensity').value)

        points: List[Tuple[float, float, float, float]] = []
        for det in msg.detections:
            label = det.id
            if not label and det.results:
                inner = getattr(det.results[0], 'hypothesis', None)
                if inner is not None:
                    label = inner.class_id
            r, h = CLASS_RADII.get(label, DEFAULT_RADIUS)
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            cz = det.bbox.center.position.z
            points.append((float(cx), float(cy), float(h * 0.5), intensity))
            for k in range(n_ring):
                ang = (2.0 * math.pi * k) / max(1, n_ring)
                px = cx + r * math.cos(ang)
                py = cy + r * math.sin(ang)
                points.append((float(px), float(py), 0.0, intensity))

        header = Header()
        header.frame_id = str(self.get_parameter('map_frame').value)
        header.stamp = self.get_clock().now().to_msg()
        cloud = _make_xyzi_cloud(header, points)
        self.pub.publish(cloud)


def _make_xyzi_cloud(header: Header,
                     points: List[Tuple[float, float, float, float]]) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.is_dense = True
    cloud.is_bigendian = False
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12,
                   datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * cloud.width
    buf = bytearray(cloud.row_step)
    fmt = '<ffff'
    off = 0
    for p in points:
        struct.pack_into(fmt, buf, off, p[0], p[1], p[2], p[3])
        off += 16
    cloud.data = bytes(buf)
    return cloud


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstaclePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
