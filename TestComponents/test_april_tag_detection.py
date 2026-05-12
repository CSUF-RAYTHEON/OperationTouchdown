import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2
import numpy as np
import depthai as dai
from UAV.Detectors.april_tag_detector import AprilTagDetector

# ---- COLOR DETECTION CONFIG ----
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


# ---- MAIN ----
with dai.Device() as device:
    print("[INFO] Camera started")

    calib = device.getCalibration()
    detector = AprilTagDetector(calib)

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

            # ---- APRILTAG DETECTION ----
            tag = detector.get_tag_detection(frame)

            if tag is not None:
                t = tag.pose_t
                cam_x = float(t[0][0])
                cam_y = float(t[1][0])
                cam_z = float(t[2][0])

                body_x = -cam_y
                body_y = cam_x
                body_z = cam_z

                print(f"CAM:  x={cam_x:.2f}, y={cam_y:.2f}, z={cam_z:.2f}")
                print(f"BODY: x={body_x:.2f}, y={body_y:.2f}, z={body_z:.2f}")
                print("------")

                # draw tag outline
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
                    f"CAM  x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                )
                cv2.putText(
                    frame,
                    f"BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                )

            cv2.imshow("AprilTag + Color Detection", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()