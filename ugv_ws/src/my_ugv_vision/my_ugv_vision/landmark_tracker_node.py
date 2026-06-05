"""Map-frame landmark deduplication and republication.

Subscribes to ``/akida/landmarks_raw`` (Detection3DArray in ``map``) and
maintains a small in-memory landmark store, deduplicated by class within a
configurable radius (default 0.5 m). Republishes:
  /akida/markers          visualization_msgs/MarkerArray
  /akida/landmark_array   vision_msgs/Detection3DArray
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Set

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

CLASS_SHAPES: Dict[str, Dict[str, object]] = {
    'traffic_cones': {
        'type': Marker.CYLINDER,
        'size': (0.30, 0.30, 0.50),
        'color': (1.0, 0.55, 0.0, 0.9),
    },
    'home-depot-bucket': {
        'type': Marker.CYLINDER,
        'size': (0.30, 0.30, 0.40),
        'color': (0.85, 0.10, 0.10, 0.9),
    },
    'Box': {
        'type': Marker.CUBE,
        'size': (0.40, 0.40, 0.40),
        'color': (0.45, 0.30, 0.15, 0.9),
    },
}
DEFAULT_SHAPE = {
    'type': Marker.CYLINDER,
    'size': (0.30, 0.30, 0.30),
    'color': (0.5, 0.5, 0.5, 0.7),
}
TEXT_ID_OFFSET = 100000


@dataclass
class Landmark:
    id: int
    class_id: str
    x: float
    y: float
    z: float
    hits: int
    last_seen_ns: int
    score_sum: float


class LandmarkTrackerNode(Node):

    def __init__(self) -> None:
        super().__init__('landmark_tracker_node')

        self.declare_parameter('raw_topic', '/akida/landmarks_raw')
        self.declare_parameter('markers_topic', '/akida/markers')
        self.declare_parameter('array_topic', '/akida/landmark_array')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('dedup_radius_m', 0.5)
        self.declare_parameter('max_landmarks', 200,
                               ParameterDescriptor(
                                   type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('republish_hz', 5.0)
        self.declare_parameter('min_score', 0.35)

        self._next_id = 1
        self._store: Dict[int, Landmark] = {}
        self._evicted_ids: Set[int] = set()

        self.create_subscription(
            Detection3DArray,
            str(self.get_parameter('raw_topic').value),
            self._on_raw, 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, str(self.get_parameter('markers_topic').value), 10)
        self.pub_array = self.create_publisher(
            Detection3DArray, str(self.get_parameter('array_topic').value), 10)

        rate = float(self.get_parameter('republish_hz').value)
        period = 1.0 / max(rate, 0.1)
        self.create_timer(period, self._publish)

        self.get_logger().info(
            f"landmark_tracker_node ready (radius="
            f"{float(self.get_parameter('dedup_radius_m').value)} m, max="
            f"{int(self.get_parameter('max_landmarks').value)})")

    def _on_raw(self, msg: Detection3DArray) -> None:
        if not msg.detections:
            return
        radius = float(self.get_parameter('dedup_radius_m').value)
        max_lm = int(self.get_parameter('max_landmarks').value)
        min_score = float(self.get_parameter('min_score').value)
        now_ns = self.get_clock().now().nanoseconds

        for det in msg.detections:
            if not det.results:
                continue
            first = det.results[0]
            inner = getattr(first, 'hypothesis', None)
            label = det.id or (inner.class_id if inner else '')
            score = float(inner.score) if inner else 0.0
            if score < min_score or not label:
                continue
            p = first.pose.pose.position
            self._integrate(label, float(p.x), float(p.y), float(p.z),
                            score, now_ns, radius)
        self._enforce_cap(max_lm)

    def _integrate(self, label: str, x: float, y: float, z: float,
                   score: float, now_ns: int, radius: float) -> None:
        best_id: Optional[int] = None
        best_d2 = radius * radius
        for lid, lm in self._store.items():
            if lm.class_id != label:
                continue
            d2 = (lm.x - x) ** 2 + (lm.y - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_id = lid

        if best_id is None:
            lm = Landmark(
                id=self._next_id, class_id=label, x=x, y=y, z=z,
                hits=1, last_seen_ns=now_ns, score_sum=score)
            self._store[lm.id] = lm
            self._next_id += 1
            return

        lm = self._store[best_id]
        n = lm.hits
        lm.x = (lm.x * n + x) / (n + 1)
        lm.y = (lm.y * n + y) / (n + 1)
        lm.z = (lm.z * n + z) / (n + 1)
        lm.hits = n + 1
        lm.last_seen_ns = now_ns
        lm.score_sum += score

    def _enforce_cap(self, cap: int) -> None:
        if len(self._store) <= cap:
            return
        items = sorted(
            self._store.values(),
            key=lambda l: (l.hits, l.last_seen_ns))
        n_evict = len(self._store) - cap
        for victim in items[:n_evict]:
            self._evicted_ids.add(victim.id)
            del self._store[victim.id]

    def _publish(self) -> None:
        map_frame = str(self.get_parameter('map_frame').value)
        now_msg = self.get_clock().now().to_msg()

        marr = MarkerArray()
        for lid in list(self._evicted_ids):
            for delta in (0, TEXT_ID_OFFSET):
                m = Marker()
                m.header.frame_id = map_frame
                m.header.stamp = now_msg
                m.ns = 'landmark'
                m.id = lid + delta
                m.action = Marker.DELETE
                marr.markers.append(m)
        self._evicted_ids.clear()

        det_arr = Detection3DArray()
        det_arr.header.frame_id = map_frame
        det_arr.header.stamp = now_msg

        for lm in self._store.values():
            shape = CLASS_SHAPES.get(lm.class_id, DEFAULT_SHAPE)
            sx, sy, sz = shape['size']  # type: ignore[misc]
            r, g, b, a = shape['color']  # type: ignore[misc]
            marker = Marker()
            marker.header.frame_id = map_frame
            marker.header.stamp = now_msg
            marker.ns = 'landmark'
            marker.id = lm.id
            marker.type = shape['type']  # type: ignore[assignment]
            marker.action = Marker.ADD
            marker.pose.position.x = lm.x
            marker.pose.position.y = lm.y
            marker.pose.position.z = lm.z + sz * 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = sx
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a
            marker.frame_locked = True
            marr.markers.append(marker)

            text = Marker()
            text.header.frame_id = map_frame
            text.header.stamp = now_msg
            text.ns = 'landmark'
            text.id = lm.id + TEXT_ID_OFFSET
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = lm.x
            text.pose.position.y = lm.y
            text.pose.position.z = lm.z + sz + 0.10
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            avg_score = lm.score_sum / max(1, lm.hits)
            text.text = f"{lm.class_id} #{lm.id} h={lm.hits} s={avg_score:.2f}"
            text.frame_locked = True
            marr.markers.append(text)

            d3 = Detection3D()
            d3.header = det_arr.header
            d3.id = lm.class_id
            hyp = ObjectHypothesisWithPose()
            inner = ObjectHypothesis()
            inner.class_id = lm.class_id
            inner.score = float(avg_score)
            hyp.hypothesis = inner
            hyp.pose.pose.position.x = lm.x
            hyp.pose.pose.position.y = lm.y
            hyp.pose.pose.position.z = lm.z
            hyp.pose.pose.orientation.w = 1.0
            d3.results.append(hyp)
            bbox3 = BoundingBox3D()
            bbox3.center.position.x = lm.x
            bbox3.center.position.y = lm.y
            bbox3.center.position.z = lm.z + sz * 0.5
            bbox3.center.orientation.w = 1.0
            bbox3.size.x = sx
            bbox3.size.y = sy
            bbox3.size.z = sz
            d3.bbox = bbox3
            det_arr.detections.append(d3)

        if marr.markers:
            self.pub_markers.publish(marr)
        self.pub_array.publish(det_arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LandmarkTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
