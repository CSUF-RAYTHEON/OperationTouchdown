"""
Akida BrainChip Object Detection Node for UGV Vision System.

Architecture:
  - The OAK-D camera streams raw RGB frames via the depthai-ros driver.
  - This node subscribes to those frames, preprocesses them, and runs them
    through a BrainChip Akida neuromorphic processor using the .fbz model.
  - Detections are published as vision_msgs/Detection2DArray and an
    annotated image is published for visualization.

Hardware:
  - OAK-D (Luxonis): connects via USB, provides raw RGB/depth streams.
  - Akida AKD1000 (BrainChip): connects via PCIe/M.2, handles inference.
  - These are fully independent units. The MyriadX VPU inside the OAK-D
    does NOT execute the .fbz model. Camera output feeds the Akida chip
    via the host CPU over PCIe/M.2.

Model architecture (ugv_object_detect_model.fbz):
  - 5-layer quantized CNN backbone (AkidaNet-derived, v2.19.1)
  - Input:  [224, 224, 3]  uint8 RGB
  - Output: [7, 7, 256]    float32 per-cell class scores
  - The image is implicitly divided into a 7×7 spatial grid.
    Each grid cell predicts a class from 256 possible classes.
    predict_classes() returns the argmax class index per cell.
  - Bounding boxes are derived from each cell's position in the grid
    (each cell covers a 32×32-pixel receptive field in the 224-px frame).

Venv:
  - akida, numpy, and opencv-python live in ugv_ws/akida_venv/.
  - The launch file sets AKIDA_VENV_SITE so the system Python used by
    ROS2 can still import rclpy while the venv provides the ML libs.
"""

import os
import sys

# ---------------------------------------------------------------------------
# Venv bootstrap — must run before any akida / numpy / cv2 imports.
# Set by the camera.launch.py via Node(additional_env={...}).
# ---------------------------------------------------------------------------
_venv_site = os.environ.get('AKIDA_VENV_SITE', '')
if _venv_site and _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from vision_msgs.msg import (  # noqa: E402
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

try:
    import akida
    AKIDA_AVAILABLE = True
except ImportError:
    AKIDA_AVAILABLE = False

# ---------------------------------------------------------------------------
# Grid geometry constants (fixed by model architecture)
# ---------------------------------------------------------------------------
# The model downsamples 224→7, so 224/7 = 32 pixels per cell in model space.
MODEL_INPUT_SIZE = 224
GRID_ROWS = 7
GRID_COLS = 7
NUM_CLASSES = 256
CELL_SIZE_PX = MODEL_INPUT_SIZE // GRID_ROWS  # 32 px per cell


class AkidaObjectDetector(Node):
    """
    ROS2 node that runs BrainChip Akida inference on OAK-D camera frames.

    Subscribes to a raw RGB image topic, runs each frame through the Akida
    spatial-classification backbone, and publishes Detection2DArray results
    (one detection per non-background grid cell above the confidence threshold)
    together with an annotated debug image.

    Published topics:
      ~/akida/detections        vision_msgs/Detection2DArray
      ~/akida/annotated_image   sensor_msgs/Image  (RGB8, toggled by parameter)
    """

    def __init__(self):
        super().__init__('akida_object_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', 'oak/rgb/image_raw')
        self.declare_parameter('confidence_threshold', 0.30)
        self.declare_parameter('background_class', -1)
        self.declare_parameter('publish_annotated_image', True)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.conf_thresh = (
            self.get_parameter('confidence_threshold').get_parameter_value().double_value
        )
        # background_class: cells predicting this class are silently skipped.
        # Set to -1 to disable background filtering (publish all detections).
        self.bg_class = (
            self.get_parameter('background_class').get_parameter_value().integer_value
        )
        self.publish_annotated = (
            self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        )

        if not AKIDA_AVAILABLE:
            self.get_logger().fatal(
                'akida Python library is not installed. '
                'Activate the venv or run:  '
                '/home/meow/Desktop/OperationTouchdown/ugv_ws/akida_venv/bin/pip install akida'
            )
            raise RuntimeError('akida not available')

        self.model = self._load_model(model_path)

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(Detection2DArray, 'akida/detections', 10)
        self.img_pub = self.create_publisher(Image, 'akida/annotated_image', 10)
        self.sub = self.create_subscription(Image, image_topic, self._image_callback, 10)

        self.get_logger().info(
            f'Akida detector ready | '
            f'input: {MODEL_INPUT_SIZE}×{MODEL_INPUT_SIZE}×3  '
            f'grid: {GRID_ROWS}×{GRID_COLS}  '
            f'classes: {NUM_CLASSES}  '
            f'conf_thresh: {self.conf_thresh}  '
            f'bg_class: {self.bg_class}  '
            f'topic: {image_topic}'
        )

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_model(self, model_path: str) -> 'akida.Model':
        if not model_path:
            self.get_logger().fatal('Parameter "model_path" is empty.')
            raise RuntimeError('model_path not set')
        if not os.path.isfile(model_path):
            self.get_logger().fatal(f'Model file not found: {model_path}')
            raise RuntimeError(f'Model file missing: {model_path}')

        self.get_logger().info(f'Loading Akida model: {model_path}')
        model = akida.Model(model_path)

        devices = akida.devices()
        if devices:
            self.get_logger().info(f'Mapping model to Akida hardware: {devices[0].desc}')
            model.map(devices[0])
        else:
            self.get_logger().warn(
                'No Akida PCIe/M.2 device detected — running in software emulation. '
                'Insert the AKD1000 board and rebuild the pipeline for hardware acceleration.'
            )

        self.get_logger().info(f'Model loaded. Input: {model.input_shape}  Output: {model.output_shape}')
        return model

    # ------------------------------------------------------------------
    # ROS image callback
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image):
        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        orig_h, orig_w = frame_rgb.shape[:2]

        # Resize to 224×224 for the model (maintains uint8 RGB)
        resized = cv2.resize(frame_rgb, (MODEL_INPUT_SIZE, MODEL_INPUT_SIZE))
        batch = np.expand_dims(resized, axis=0).astype(np.uint8)   # (1, 224, 224, 3)

        try:
            # predict()        → (1, 7, 7, 256) float32 per-cell scores
            # predict_classes() → (49,)          int      argmax class per cell
            score_map = self.model.predict(batch)[0]               # (7, 7, 256)
            class_map = self.model.predict_classes(batch)           # (49,) flat
        except Exception as exc:
            self.get_logger().error(f'Akida inference error: {exc}')
            return

        class_grid = class_map.reshape(GRID_ROWS, GRID_COLS)       # (7, 7)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                cls = int(class_grid[row, col])

                # Skip background cells
                if self.bg_class >= 0 and cls == self.bg_class:
                    continue

                # Confidence = normalised score for the winning class
                cell_scores = score_map[row, col]                   # (256,)
                raw_conf = float(cell_scores[cls])
                # Scores are already positive floats; normalise by the sum
                # so confidence is comparable across cells.
                score_sum = float(cell_scores.sum())
                conf = raw_conf / score_sum if score_sum > 0.0 else 0.0

                if conf < self.conf_thresh:
                    continue

                # Map cell (row, col) → pixel bbox in original frame
                # Each cell covers 1/7 of the 224-px model frame
                scale_x = orig_w / MODEL_INPUT_SIZE
                scale_y = orig_h / MODEL_INPUT_SIZE

                x1 = col * CELL_SIZE_PX * scale_x
                y1 = row * CELL_SIZE_PX * scale_y
                x2 = (col + 1) * CELL_SIZE_PX * scale_x
                y2 = (row + 1) * CELL_SIZE_PX * scale_y

                det_array.detections.append(
                    self._make_detection(x1, y1, x2, y2, conf, cls)
                )

        self.det_pub.publish(det_array)

        if self.publish_annotated:
            annotated = self._draw_detections(frame_rgb, det_array)
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
            ann_msg.header = msg.header
            self.img_pub.publish(ann_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_detection(
        x1: float, y1: float, x2: float, y2: float,
        conf: float, cls: int,
    ) -> Detection2D:
        det = Detection2D()
        bbox = BoundingBox2D()
        # vision_msgs/BoundingBox2D uses geometry_msgs/Pose2D for centre
        bbox.center.x = (x1 + x2) / 2.0
        bbox.center.y = (y1 + y2) / 2.0
        bbox.size_x = x2 - x1
        bbox.size_y = y2 - y1
        det.bbox = bbox
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(cls)
        hyp.hypothesis.score = conf
        det.results.append(hyp)
        return det

    def _draw_detections(self, frame: np.ndarray, det_array: Detection2DArray) -> np.ndarray:
        annotated = frame.copy()
        for det in det_array.detections:
            if det.bbox.size_x > 0 and det.bbox.size_y > 0:
                cx = int(det.bbox.center.x)
                cy = int(det.bbox.center.y)
                half_w = int(det.bbox.size_x / 2)
                half_h = int(det.bbox.size_y / 2)
                x1 = max(cx - half_w, 0)
                y1 = max(cy - half_h, 0)
                x2 = min(cx + half_w, frame.shape[1] - 1)
                y2 = min(cy + half_h, frame.shape[0] - 1)
                if det.results:
                    hyp = det.results[0]
                    label = f'cls:{hyp.hypothesis.class_id}  {hyp.hypothesis.score:.2f}'
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        annotated, label, (x1, max(y1 - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                    )
        # Draw the 7×7 grid overlay so operators can see cell boundaries
        h, w = frame.shape[:2]
        cell_w = w / GRID_COLS
        cell_h = h / GRID_ROWS
        for i in range(1, GRID_COLS):
            cv2.line(annotated, (int(i * cell_w), 0), (int(i * cell_w), h), (80, 80, 80), 1)
        for j in range(1, GRID_ROWS):
            cv2.line(annotated, (0, int(j * cell_h)), (w, int(j * cell_h)), (80, 80, 80), 1)
        return annotated


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AkidaObjectDetector()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
