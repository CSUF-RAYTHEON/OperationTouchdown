# READY-TO-GO runner  —  nested 36h11 detection on the OAK-D / DepthAI camera.
#
# Detects BOTH layers of a nested AprilTag board (outer + inner) and prints the
# 3-D position of each in the camera frame and the drone body frame.  This is
# the on-drone / hardware path: it pulls real camera intrinsics from the OAK-D
# calibration so pose (x, y, z in meters) is accurate.
#
# For the precision-landing controller, NestedArucoDetector.get_landing_target()
# returns the INNER tag when visible (best for the final close-range step) and
# otherwise the OUTER tag (long-range acquisition).
#
# Run on the drone / a machine with an OAK-D attached:
#     python3 TestComponents/test_nested_36h11_detection.py
# Press 'q' in the preview window to quit.
#
# Configure the board IDs and physical tag sizes in
# UAV/Detectors/nested_aruco_detector.py (OUTER_TAG_ID, OUTER_TAG_SIZE, ...).

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2
import numpy as np
import depthai as dai

from UAV.Detectors.nested_aruco_detector import NestedArucoDetector


# Camera resolution — matches test_36h11_detection.py (sufficient at 1–4 m).
# Raise to (640, 640) if you need to resolve the small inner tag from farther.
CAMERA_RESOLUTION = (300, 300)

# Draw colors (BGR)
COLOR_OUTER = (0, 255, 0)      # green
COLOR_INNER = (0, 165, 255)    # orange


def _intrinsics_from_calibration(calib, width, height):
    """Pull the camera matrix and distortion coefficients for CAM_A at the
    given frame size from the OAK-D calibration handler."""
    camera_matrix = np.array(
        calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height),
        dtype=np.float64,
    )
    dist_coeffs = np.array(
        calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A),
        dtype=np.float64,
    )
    return camera_matrix, dist_coeffs


def main():
    with dai.Device() as device:
        print("[INFO] Camera started")

        calib = device.getCalibration()

        # Intrinsics depend on the output frame size, so build them to match
        # CAMERA_RESOLUTION and hand them to the detector for accurate pose.
        cam_mtx, dist = _intrinsics_from_calibration(
            calib, CAMERA_RESOLUTION[0], CAMERA_RESOLUTION[1]
        )
        detector = NestedArucoDetector(camera_matrix=cam_mtx, dist_coeffs=dist)

        with dai.Pipeline(device) as pipeline:
            cam_rgb = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A
            )
            rgb_out = cam_rgb.requestOutput(
                size=CAMERA_RESOLUTION,
                type=dai.ImgFrame.Type.NV12,
                fps=30,
            )
            q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)

            pipeline.start()

            while pipeline.isRunning():
                in_rgb = q_rgb.get()
                if in_rgb is None:
                    continue

                frame = in_rgb.getCvFrame()

                detections = detector.detect(frame)

                outer_seen = False
                inner_seen = False

                for det in detections:
                    color = COLOR_OUTER if det.layer == "outer" else COLOR_INNER
                    outer_seen |= det.layer == "outer"
                    inner_seen |= det.layer == "inner"

                    # Pose in camera frame -> drone body frame (FRD-ish), the
                    # same convention used by test_36h11_detection.py.
                    t = det.pose_t
                    cam_x, cam_y, cam_z = float(t[0][0]), float(t[1][0]), float(t[2][0])
                    body_x, body_y, body_z = -cam_y, cam_x, cam_z

                    print(
                        f"[{det.layer.upper():5s} id={det.tag_id}] "
                        f"CAM x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f} | "
                        f"BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}"
                    )

                    corners = det.corners.astype(int)
                    for i in range(4):
                        cv2.line(
                            frame,
                            tuple(corners[i]),
                            tuple(corners[(i + 1) % 4]),
                            color, 2,
                        )
                    cv2.circle(frame, tuple(det.center.astype(int)), 4, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"{det.layer.upper()} id{det.tag_id} z={cam_z:.2f}",
                        (corners[0][0], corners[0][1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1,
                    )

                if outer_seen and inner_seen:
                    status, scolor = "BOTH LAYERS", COLOR_OUTER
                elif outer_seen:
                    status, scolor = "OUTER only", COLOR_INNER
                elif inner_seen:
                    status, scolor = "INNER only", COLOR_INNER
                else:
                    status, scolor = "no target", (0, 0, 255)
                cv2.putText(
                    frame, f"STATUS: {status}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, scolor, 1,
                )

                cv2.imshow("Nested 36h11 (OAK-D)", frame)
                if cv2.waitKey(1) == ord('q'):
                    break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
