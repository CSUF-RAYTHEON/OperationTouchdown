"""ROS 2 node that runs the BrainChip AKD1000 YOLO model on RGB frames.

Subscribes to ``/oak/rgb/image_raw`` (best-effort), runs the Akida-deployed
YOLOv2 head, publishes ``vision_msgs/Detection2DArray`` (rescaled back to the
original RGB resolution), an annotated image, and inference latency.

Akida ``forward()`` is single-threaded; we serialize with a Lock and drop new
frames if the previous one is still in flight (matches sensor-data QoS).
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import Counter
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from my_ugv_vision.yolo_decode import (
    decode_yolo_v2,
    letterbox,
    nms,
    unletterbox_boxes,
)

CLASS_COLORS_BGR: Dict[str, Tuple[int, int, int]] = {
    'traffic_cones': (0, 140, 255),
    'home-depot-bucket': (0, 0, 220),
    'Box': (45, 82, 160),
}
DEFAULT_COLOR = (0, 255, 0)


class AkidaYoloNode(Node):

    def __init__(self) -> None:
        super().__init__('akida_yolo_node')

        self.declare_parameter(
            'model_path', '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Absolute path to the .fbz Akida model. If empty, '
                'looks next to the package share models/ dir.'))
        self.declare_parameter(
            'metadata_path', '',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Absolute path to model_metadata.json (labels, '
                'anchors). If empty, sits next to model_path.'))
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('detections_topic', '/akida/detections')
        self.declare_parameter('annotated_topic', '/akida/image_annotated')
        self.declare_parameter('latency_topic', '/akida/inference_latency_ms')
        self.declare_parameter('objects_topic', '/akida/detected_objects')
        self.declare_parameter('score_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('diagnostics_period_s', 1.0)
        self.declare_parameter(
            'labels_override', [''],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Optional class label override; empty = use metadata.'))
        self.declare_parameter(
            'anchors_flat_override', [0.0],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='Optional anchors override flattened [aw0,ah0,...]; '
                'all-zeros = use metadata.'))

        self.score_thresh = float(self.get_parameter('score_threshold').value)
        self.iou_thresh = float(self.get_parameter('iou_threshold').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)

        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            from ament_index_python.packages import get_package_share_directory
            model_path = os.path.join(
                get_package_share_directory('my_ugv_vision'),
                'models', 'ugv_yolo_akida.fbz')
        if not os.path.isfile(model_path):
            self.get_logger().error(
                f"Akida model not found at {model_path}; node will not run inference.")

        metadata_path = str(self.get_parameter('metadata_path').value).strip()
        if not metadata_path:
            metadata_path = os.path.join(
                os.path.dirname(model_path), 'model_metadata.json')

        self.labels, anchors_np, self.input_size, self.grid = \
            self._load_metadata(metadata_path)

        labels_override = list(self.get_parameter('labels_override').value)
        if labels_override and any(s for s in labels_override):
            self.labels = labels_override
            self.get_logger().info(f"Using label override: {self.labels}")

        anchors_override = list(self.get_parameter('anchors_flat_override').value)
        if anchors_override and any(abs(v) > 1e-9 for v in anchors_override):
            self.anchors = np.asarray(anchors_override,
                                      dtype=np.float32).reshape(-1, 2)
            self.get_logger().info(
                f"Using anchor override (shape={self.anchors.shape})")
        else:
            self.anchors = anchors_np

        self.num_classes = len(self.labels)
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._busy = False
        self.frames_seen = 0
        self.frames_processed = 0
        self.frames_dropped = 0
        self.last_latency_ms = 0.0
        self.last_num_detections = 0

        self.device_kind = 'unknown'
        self.model = None
        self._try_load_akida(model_path)

        self.pub_dets = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value), 10)
        self.pub_annot = self.create_publisher(
            Image, str(self.get_parameter('annotated_topic').value), 10)
        self.pub_latency = self.create_publisher(
            Float32, str(self.get_parameter('latency_topic').value), 10)
        self.pub_objects = self.create_publisher(
            String, str(self.get_parameter('objects_topic').value), 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.sub_image = self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)

        diag_period = float(self.get_parameter('diagnostics_period_s').value)
        if diag_period > 0.0:
            self.create_timer(diag_period, self._publish_diagnostics)

        self.get_logger().info(
            f"akida_yolo_node ready. labels={self.labels} grid={self.grid} "
            f"input={self.input_size} anchors={self.anchors.shape[0]} "
            f"device={self.device_kind}")

    def _load_metadata(self, path: str):
        labels = ['Box', 'home-depot-bucket', 'traffic_cones']
        anchors = np.array(
            [[0.37449, 0.44152], [0.88506, 0.93953], [1.87466, 1.40808],
             [2.14199, 2.77736], [5.05428, 5.53107]], dtype=np.float32)
        input_size = (224, 224)
        grid = (7, 7)
        if not os.path.isfile(path):
            self.get_logger().warn(
                f"model_metadata.json not at {path}; using hard-coded defaults.")
            return labels, anchors, input_size, grid
        with open(path, 'r') as f:
            meta = json.load(f)
        labels = list(meta.get('labels', labels))
        anchors_raw = meta.get('anchors')
        if anchors_raw is not None:
            anchors = np.asarray(anchors_raw, dtype=np.float32).reshape(-1, 2)
        cfg = meta.get('config', {})
        ishape = cfg.get('input_shape')
        if ishape and len(ishape) >= 2:
            input_size = (int(ishape[1]), int(ishape[0]))
        gsize = cfg.get('grid_size')
        if gsize and len(gsize) == 2:
            grid = (int(gsize[1]), int(gsize[0]))
        return labels, anchors, input_size, grid

    def _try_load_akida(self, model_path: str) -> None:
        try:
            from akida import Model, devices  # type: ignore
        except Exception as exc:
            self.get_logger().error(
                f"Failed to import 'akida' package ({exc}); install with "
                "'pip install akida akida-models'. Inference disabled.")
            return

        if not os.path.isfile(model_path):
            return

        try:
            self.model = Model(model_path)
        except Exception as exc:
            self.get_logger().error(f"Failed to load model {model_path}: {exc}")
            self.model = None
            return

        dev_list = []
        try:
            dev_list = list(devices())
        except Exception as exc:
            self.get_logger().warn(f"devices() raised: {exc}")

        # Fast path: try to map the whole model onto the AKD1000. If the model
        # cannot be fully mapped to silicon (e.g. the final detection layer's
        # act_step exceeds the hardware's 16-bit limit), fall back to Akida's
        # software backend so detection still works while the model is being
        # re-quantized for hardware. An unmapped Model.forward() runs in SW.
        self.device_kind = 'software'
        if dev_list:
            device = dev_list[0]
            self.get_logger().info(
                f"Akida hardware found: {device}. Mapping model...")
            try:
                self.model.map(device, hw_only=False)
                self.device_kind = 'hardware'
            except Exception as exc:
                self.get_logger().warn(
                    f"Hardware map failed ({exc}). Falling back to SOFTWARE "
                    f"inference on the CPU. Detection still works (~6-7 FPS); "
                    f"re-quantize the final layer to move it onto the AKD1000.")
                # A failed map() can leave the model in a partial state; reload
                # a clean copy for software execution.
                try:
                    self.model = Model(model_path)
                except Exception as exc2:
                    self.get_logger().error(
                        f"Failed to reload model for SW fallback: {exc2}")
                    self.model = None
                    return
        else:
            self.get_logger().warn(
                "No Akida hardware visible; using SOFTWARE inference on the "
                "CPU. Check 'akida devices' and the PCIe driver to use the chip.")

        try:
            self.get_logger().info("Akida model summary:\n" + str(self.model.summary()))
        except Exception:
            pass

        # Probe one forward pass so any execution problem surfaces once here
        # instead of on every camera frame.
        try:
            input_w, input_h = self.input_size
            self.model.forward(
                np.zeros((1, input_h, input_w, 3), dtype=np.uint8))
        except Exception as exc:
            self.get_logger().error(
                f"Akida inference probe failed: {exc}. Detection disabled "
                f"(model IP is {getattr(self.model, 'ip_version', '?')}).")
            self.model = None
            return
        self.get_logger().info(
            f"Akida inference ready (device_kind={self.device_kind}).")

    def _on_image(self, msg: Image) -> None:
        self.frames_seen += 1
        if self.model is None:
            return
        if not self._lock.acquire(blocking=False):
            self.frames_dropped += 1
            return
        try:
            if self._busy:
                self.frames_dropped += 1
                return
            self._busy = True
        finally:
            self._lock.release()

        try:
            self._process(msg)
        except Exception as exc:
            self.get_logger().error(f"inference failure: {exc}")
        finally:
            with self._lock:
                self._busy = False

    def _process(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f"cv_bridge convert failed: {exc}")
            return

        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        orig_h, orig_w = rgb.shape[:2]
        input_w, input_h = self.input_size

        padded, scale, pad_x, pad_y = letterbox(rgb, input_w, input_h)
        batch = padded[None, ...].astype(np.uint8)

        t0 = time.perf_counter()
        try:
            raw = self.model.forward(batch)
        except Exception as exc:
            self.get_logger().error(f"model.forward failed: {exc}")
            return
        t1 = time.perf_counter()
        self.last_latency_ms = (t1 - t0) * 1000.0
        self.frames_processed += 1

        raw_np = np.asarray(raw, dtype=np.float32)
        if raw_np.ndim == 2:
            raw_np = raw_np.reshape(1, self.grid[1], self.grid[0], -1)

        decoded = decode_yolo_v2(
            raw_np, self.anchors, self.num_classes,
            input_size=self.input_size, grid=self.grid,
            score_thresh=self.score_thresh)
        pruned = nms(decoded, self.iou_thresh)
        remapped = unletterbox_boxes(pruned, orig_w, orig_h, input_w, input_h)
        self.last_num_detections = len(remapped)

        det_arr = Detection2DArray()
        det_arr.header = msg.header
        for x1, y1, x2, y2, score, cls in remapped:
            det = Detection2D()
            det.header = msg.header
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            bbox = BoundingBox2D()
            bbox.center.position.x = float(cx)
            bbox.center.position.y = float(cy)
            bbox.center.theta = 0.0
            bbox.size_x = float(max(0.0, x2 - x1))
            bbox.size_y = float(max(0.0, y2 - y1))
            det.bbox = bbox
            label = self.labels[cls] if 0 <= cls < len(self.labels) else str(cls)
            hyp = ObjectHypothesisWithPose()
            inner = ObjectHypothesis()
            inner.class_id = label
            inner.score = float(score)
            hyp.hypothesis = inner
            det.results.append(hyp)
            det.id = label
            det_arr.detections.append(det)
        self.pub_dets.publish(det_arr)

        lat_msg = Float32()
        lat_msg.data = float(self.last_latency_ms)
        self.pub_latency.publish(lat_msg)

        # Per-class text summary for the Foxglove panel. Counts multiple
        # instances of the same class, e.g. "3x traffic_cones, 1x Box".
        if remapped:
            counts = Counter(
                self.labels[c] if 0 <= c < len(self.labels) else str(c)
                for *_box, _score, c in remapped)
            summary = ', '.join(f"{n}x {name}" for name, n in counts.items())
        else:
            summary = 'none'
        obj_msg = String()
        obj_msg.data = summary
        self.pub_objects.publish(obj_msg)

        if self.publish_annotated and self.pub_annot.get_subscription_count() >= 0:
            self._publish_annotated(bgr, remapped, msg)

        del scale, pad_x, pad_y

    def _publish_annotated(self, bgr: np.ndarray,
                           boxes: List[Tuple[float, float, float, float, float, int]],
                           src_msg: Image) -> None:
        import cv2
        annotated = bgr.copy()
        for x1, y1, x2, y2, score, cls in boxes:
            label = self.labels[cls] if 0 <= cls < len(self.labels) else str(cls)
            color = CLASS_COLORS_BGR.get(label, DEFAULT_COLOR)
            p1 = (int(round(x1)), int(round(y1)))
            p2 = (int(round(x2)), int(round(y2)))
            cv2.rectangle(annotated, p1, p2, color, 2)
            text = f"{label} {score:.2f}"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty1 = max(0, p1[1] - th - 4)
            cv2.rectangle(annotated, (p1[0], ty1), (p1[0] + tw + 4, p1[1]), color, -1)
            cv2.putText(annotated, text, (p1[0] + 2, p1[1] - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = src_msg.header
            self.pub_annot.publish(out_msg)
        except Exception as exc:
            self.get_logger().warn(f"annotated publish failed: {exc}")

    def _publish_diagnostics(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'my_ugv_vision: akida_yolo_node'
        status.hardware_id = 'AKD1000'
        if self.model is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'model not loaded'
        elif self.device_kind == 'software':
            status.level = DiagnosticStatus.WARN
            status.message = 'running in software (CPU) — AKD1000 not in use'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        status.values = [
            KeyValue(key='device_kind', value=self.device_kind),
            KeyValue(key='frames_seen', value=str(self.frames_seen)),
            KeyValue(key='frames_processed', value=str(self.frames_processed)),
            KeyValue(key='frames_dropped', value=str(self.frames_dropped)),
            KeyValue(key='last_latency_ms', value=f"{self.last_latency_ms:.2f}"),
            KeyValue(key='last_num_detections',
                     value=str(self.last_num_detections)),
            KeyValue(key='labels', value=','.join(self.labels)),
        ]
        msg.status.append(status)
        self.pub_diag.publish(msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = AkidaYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
