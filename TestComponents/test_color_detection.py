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


def _detect_colors(frame: np.ndarray) -> list:
    """
    Detect pink, blue, and yellow regions in *frame* (BGR).

    Returns a list of (color_name, x, y, w, h) tuples for every qualifying
    contour.  Nothing is drawn here — drawing is done separately after all
    detections are complete so that annotation pixels never contaminate the
    input to any detector.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = []

    for color_name, ranges in _COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for (lo, hi) in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Light morphological cleanup to remove noise and fill small holes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < _MIN_CONTOUR_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            results.append((color_name, x, y, w, h))

    return results


def _draw_colors(frame: np.ndarray, detections: list) -> None:
    """Draw bounding rectangles and labels for color detections onto *frame*."""
    for (color_name, x, y, w, h) in detections:
        bgr = _COLOR_BGR[color_name]
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

# Semi-transparent pink fill drawn behind the marker polygon
_PINK_TAG_FILL_BGR   = (180, 105, 255)
_PINK_TAG_FILL_ALPHA = 0.35

# HUD text color for pink-tag readouts
_PINK_TAG_TEXT_BGR = (180, 105, 255)


def _tag_is_pink(frame: np.ndarray, corners: np.ndarray) -> bool:
    """
    Return True if the interior of the detected tag polygon in the ORIGINAL
    (un-remapped) *frame* is predominantly hot/neon pink.

    Threshold: >10% of the polygon area is pink.  This is intentionally low
    because only the "white" squares of the marker are pink — the black
    squares and border are not — so a fully pink-variant marker has roughly
    30–40% pink coverage inside the polygon.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    region_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    cv2.fillPoly(region_mask, [corners.reshape((-1, 1, 2))], 255)

    pink_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in _PINK_TAG_HSV_RANGES:
        pink_mask = cv2.bitwise_or(pink_mask, cv2.inRange(hsv, lo, hi))

    inside_total = int(np.sum(region_mask > 0))
    inside_pink  = int(np.sum(cv2.bitwise_and(pink_mask, region_mask) > 0))

    if inside_total == 0:
        return False
    return (inside_pink / inside_total) > 0.10


def _replace_pink_with_white(frame: np.ndarray) -> np.ndarray:
    """
    Return a copy of *frame* (BGR) where every hot/neon pink pixel has been
    replaced with white (255, 255, 255).

    This remaps the "white" regions of a pink-variant AprilTag marker back to
    true white so that the standard AprilTag detector can process the marker.
    The black squares are unaffected.

    A light morphological dilation is applied to the pink mask before
    substitution to capture soft / anti-aliased edge pixels, preventing a
    dark fringe from forming around the remapped squares.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in _PINK_TAG_HSV_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.dilate(mask, kernel, iterations=1)

    modified = frame.copy()
    modified[mask > 0] = [255, 255, 255]
    return modified


class PinkAprilTagDetector:
    """
    Detects AprilTag markers whose traditionally white squares are printed in
    hot/neon pink instead.

    Internally wraps a standard AprilTagDetector.  Before every detection call
    the input frame is passed through _replace_pink_with_white() so that pink
    regions become white and the detector sees a normal black-and-white pattern.
    The full 27-pass preprocessing pipeline defined in AprilTagDetector is
    therefore applied identically — only the colour remapping step is inserted
    before that pipeline runs.

    _tag_is_pink() is called on the ORIGINAL frame after detection.  If the
    tag region is not predominantly pink the result is discarded, preventing a
    standard white marker from being counted here when both marker types are
    present in the same scene.
    """

    def __init__(self, calibration_handler):
        self._detector = AprilTagDetector(calibration_handler)

    def get_tag_detection(self, frame):
        """
        Convert pink → white on a frame copy, run the full AprilTag pipeline,
        then verify the result is actually a pink-variant marker.
        Returns the raw Detection object, or None.
        """
        if frame is None:
            return None
        remapped = _replace_pink_with_white(frame)
        tag = self._detector.get_tag_detection(remapped)
        if tag is None:
            return None
        if not _tag_is_pink(frame, tag.corners.astype(int)):
            return None
        return tag


# =============================================================================
# MAIN
# =============================================================================
with dai.Device() as device:
    print("[INFO] Camera started")

    calib = device.getCalibration()
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

            # ------------------------------------------------------------------
            # DETECTION PHASE — run all detectors on the clean, unmodified frame
            # before any annotation pixels are written to it.
            # ------------------------------------------------------------------
            color_hits = _detect_colors(frame)
            pink_tag   = pink_detector.get_tag_detection(frame)

            # ------------------------------------------------------------------
            # DRAW PHASE — annotate the frame only after all detections are done
            # ------------------------------------------------------------------

            # Color blobs
            _draw_colors(frame, color_hits)

            # Pink AprilTag
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

                # Semi-transparent pink background fill
                overlay = frame.copy()
                cv2.fillPoly(overlay, [corners.reshape((-1, 1, 2))], _PINK_TAG_FILL_BGR)
                cv2.addWeighted(overlay, _PINK_TAG_FILL_ALPHA,
                                frame,   1.0 - _PINK_TAG_FILL_ALPHA, 0, frame)

                # Green corner outline on top of the fill
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
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _PINK_TAG_TEXT_BGR, 1
                )
                cv2.putText(
                    frame,
                    f"PINK BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}",
                    (10, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _PINK_TAG_TEXT_BGR, 1
                )

            cv2.imshow("Color + Pink Tag Detection", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
