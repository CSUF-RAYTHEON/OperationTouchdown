import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2
import numpy as np
import depthai as dai
from UAV.Detectors.april_tag_detector import AprilTagDetector


# =============================================================================
# COLOR DETECTION
# =============================================================================
# HSV ranges for each target color.
# OpenCV HSV: H in [0, 180], S and V in [0, 255].
#
# Pink wraps the hue wheel so two ranges are needed (upper-red / magenta band).
_COLOR_RANGES = {
    "pink": [
        (np.array([155, 50,  80]),  np.array([180, 255, 255])),
        (np.array([0,   50,  80]),  np.array([10,  180, 255])),
    ],
    "blue": [
        (np.array([100, 100, 50]),  np.array([130, 255, 255])),
    ],
    "yellow": [
        (np.array([18,  100, 100]), np.array([35,  255, 255])),
    ],
}

# Draw color: (B, G, R)
_COLOR_BGR = {
    "pink":   (180, 105, 255),
    "blue":   (255, 100,   0),
    "yellow": (  0, 220, 220),
}

# Minimum contour area (px²) to suppress noise blobs
_MIN_CONTOUR_AREA = 400


def detect_colors(frame: np.ndarray) -> None:
    """
    Detect pink, blue, and yellow regions in *frame* (BGR) and draw
    bounding rectangles + labels directly onto the frame.

    This function is intentionally independent of the AprilTag detector —
    it operates purely on color information and does not affect tag detection.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    for color_name, ranges in _COLOR_RANGES.items():
        # Combine all HSV ranges for this color into one mask
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lo, hi) in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Light morphological cleanup to remove noise and fill small holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        bgr = _COLOR_BGR[color_name]

        for cnt in contours:
            if cv2.contourArea(cnt) < _MIN_CONTOUR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
            cv2.putText(
                frame,
                color_name,
                (x, max(y - 6, 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1,
            )


# =============================================================================
# PINK APRILTAG DETECTION
# =============================================================================
# Hot/neon pink HSV ranges used to identify the "white" regions of a
# pink-variant AprilTag marker.  These are tighter and more saturated than
# the general-purpose pink color-detection ranges above so that ordinary
# skin tones or light-pink objects do not accidentally trigger tag detection.
#
# Two ranges are needed because hot/neon pink straddles the hue-wheel seam:
#   Range 1 — upper magenta/rose band  (H 150–180)
#   Range 2 — lower red/rose wrap-around (H 0–10)
_PINK_TAG_HSV_RANGES = [
    (np.array([150, 120, 100]), np.array([180, 255, 255])),
    (np.array([0,   120, 100]), np.array([10,  255, 255])),
]

# Outline color for a detected pink tag — green, matching standard tags
_PINK_TAG_OUTLINE_BGR = (0, 255, 0)

# Semi-transparent pink fill drawn behind the marker polygon to visually
# indicate that this is the pink-background variant.
# The green corner outline is drawn on top so the ArUco detection is still clear.
_PINK_TAG_FILL_BGR   = (180, 105, 255)
_PINK_TAG_FILL_ALPHA = 0.35            # 0.0 = invisible, 1.0 = fully opaque

# Text color for pink-tag HUD lines (kept pink to distinguish from standard-tag
# text even though both outlines are green)
_PINK_TAG_TEXT_BGR = (180, 105, 255)


def _replace_pink_with_white(frame: np.ndarray) -> np.ndarray:
    """
    Return a copy of *frame* (BGR) where every hot/neon pink pixel has been
    replaced with white (255, 255, 255).

    This remaps the "white" regions of a pink-variant AprilTag marker back to
    true white so that the standard AprilTag detector — which expects a
    black-and-white pattern — can process the marker without modification.
    The black squares of the marker are unaffected.

    A light morphological dilation is applied to the pink mask before
    substitution so that anti-aliased or slightly under-saturated edge pixels
    are also captured, preventing a dark fringe from forming around what
    should be white squares.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in _PINK_TAG_HSV_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    # Dilate slightly to capture soft / anti-aliased pink edges
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.dilate(mask, kernel, iterations=1)

    modified = frame.copy()
    modified[mask > 0] = [255, 255, 255]
    return modified


class PinkAprilTagDetector:
    """
    Detects AprilTag markers whose traditionally white squares are printed in
    hot/neon pink instead.

    Internally this wraps a standard AprilTagDetector.  Before every detection
    call the input frame is passed through _replace_pink_with_white() so that
    pink regions become white, and the detector sees a normal black-and-white
    pattern.  This means the full 27-pass preprocessing pipeline defined in
    AprilTagDetector is applied identically — only the colour remapping step
    is inserted before that pipeline runs.

    Usage is identical to AprilTagDetector.get_tag_detection():
        pink_detector = PinkAprilTagDetector(calib)
        tag = pink_detector.get_tag_detection(frame)
    """

    def __init__(self, calibration_handler):
        self._detector = AprilTagDetector(calibration_handler)

    def get_tag_detection(self, frame):
        """
        Convert pink → white in a frame copy, then run the full AprilTag
        detection pipeline.  Returns the raw Detection object for the target
        tag ID, or None if not found.
        """
        if frame is None:
            return None
        remapped = _replace_pink_with_white(frame)
        return self._detector.get_tag_detection(remapped)


# =============================================================================
# MAIN
# =============================================================================
with dai.Device() as device:
    print("[INFO] Camera started")

    calib = device.getCalibration()

    # Standard white AprilTag detector
    detector = AprilTagDetector(calib)

    # Pink-variant AprilTag detector (same preprocessing, pink→white remapping)
    pink_detector = PinkAprilTagDetector(calib)

    # ---- PIPELINE ----
    with dai.Pipeline(device) as pipeline:

        cam_rgb = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        rgb_out = cam_rgb.requestOutput(
            size=(300, 300),
            type=dai.ImgFrame.Type.NV12,
            fps=30
        )

        q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)

        pipeline.start()

        while pipeline.isRunning():

            in_rgb = q_rgb.get()
            if in_rgb is None:
                continue

            frame = in_rgb.getCvFrame()

            # ---- COLOR DETECTION ----
            detect_colors(frame)

            # ---- STANDARD APRILTAG DETECTION ----
            tag = detector.get_tag_detection(frame)

            if tag is not None:
                t = tag.pose_t
                cam_x = float(t[0][0])
                cam_y = float(t[1][0])
                cam_z = float(t[2][0])

                body_x = -cam_y
                body_y = cam_x
                body_z = cam_z

                print(f"[TAG]  CAM:  x={cam_x:.2f}, y={cam_y:.2f}, z={cam_z:.2f}")
                print(f"[TAG]  BODY: x={body_x:.2f}, y={body_y:.2f}, z={body_z:.2f}")
                print("------")

                corners = tag.corners.astype(int)
                for i in range(4):
                    cv2.line(frame,
                             tuple(corners[i]),
                             tuple(corners[(i + 1) % 4]),
                             (0, 255, 0), 2)

                center = tuple(tag.center.astype(int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                cv2.putText(
                    frame,
                    f"TAG  CAM  x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                )
                cv2.putText(
                    frame,
                    f"TAG  BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}",
                    (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                )

            # ---- PINK APRILTAG DETECTION ----
            pink_tag = pink_detector.get_tag_detection(frame)

            if pink_tag is not None:
                t = pink_tag.pose_t
                cam_x = float(t[0][0])
                cam_y = float(t[1][0])
                cam_z = float(t[2][0])

                body_x = -cam_y
                body_y = cam_x
                body_z = cam_z

                print(f"[PINK] CAM:  x={cam_x:.2f}, y={cam_y:.2f}, z={cam_z:.2f}")
                print(f"[PINK] BODY: x={body_x:.2f}, y={body_y:.2f}, z={body_z:.2f}")
                print("------")

                corners = pink_tag.corners.astype(int)

                # Semi-transparent pink background fill — visually encases the
                # marker so it is clearly identified as the pink-background variant
                overlay = frame.copy()
                cv2.fillPoly(overlay, [corners.reshape((-1, 1, 2))], _PINK_TAG_FILL_BGR)
                cv2.addWeighted(overlay, _PINK_TAG_FILL_ALPHA,
                                frame,   1.0 - _PINK_TAG_FILL_ALPHA, 0, frame)

                # Green corner outline drawn on top of the fill — matches the
                # standard tag outline style and shows the ArUco detection inside
                for i in range(4):
                    cv2.line(frame,
                             tuple(corners[i]),
                             tuple(corners[(i + 1) % 4]),
                             _PINK_TAG_OUTLINE_BGR, 2)

                center = tuple(pink_tag.center.astype(int))
                cv2.circle(frame, center, 5, _PINK_TAG_OUTLINE_BGR, -1)

                cv2.putText(
                    frame,
                    f"PINK CAM  x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}",
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _PINK_TAG_TEXT_BGR, 1
                )
                cv2.putText(
                    frame,
                    f"PINK BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}",
                    (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _PINK_TAG_TEXT_BGR, 1
                )

            cv2.imshow("AprilTag + Color Detection", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()