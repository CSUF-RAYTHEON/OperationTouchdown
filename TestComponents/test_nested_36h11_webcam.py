# TESTING runner  —  nested 36h11 detection off-drone (laptop webcam or PNG).
#
# No OAK-D / DepthAI required.  Use this to verify the nested two-pass
# detection works on the generated boards before deploying to the drone.
#
# Without camera intrinsics the detector returns corners + center only (no
# 3-D pose), which is all you need to confirm both layers are detected.
#
# Examples:
#   # live laptop webcam (device 0), outer id 77 / inner id 12
#   python3 TestComponents/test_nested_36h11_webcam.py
#
#   # a different webcam and a different board's outer id
#   python3 TestComponents/test_nested_36h11_webcam.py --camera 1 --outer-id 349
#
#   # run on a generated PNG straight from the Nested_AprilTags repo
#   python3 TestComponents/test_nested_36h11_webcam.py \
#       --image ~/Downloads/Nested_AprilTags/test_apriltag_boards/board_outer_349_inner_12.png \
#       --outer-id 349
#
# Press 'q' to quit (webcam) or any key to close (single image).

import sys
import os
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2

from UAV.Detectors.nested_aruco_detector import (
    NestedArucoDetector,
    OUTER_TAG_ID,
    INNER_TAG_ID,
)

COLOR_OUTER = (0, 255, 0)      # green
COLOR_INNER = (0, 165, 255)    # orange


def draw_detections(frame, detections):
    """Draw each detected layer and a status banner.  Returns (outer, inner)
    booleans for what was seen."""
    outer_seen = inner_seen = False
    for det in detections:
        color = COLOR_OUTER if det.layer == "outer" else COLOR_INNER
        outer_seen |= det.layer == "outer"
        inner_seen |= det.layer == "inner"

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
            f"{det.layer.upper()} id{det.tag_id}",
            (corners[0][0], corners[0][1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
        )

    if outer_seen and inner_seen:
        status, scolor = "BOTH LAYERS TRACKED", COLOR_OUTER
    elif outer_seen:
        status, scolor = "OUTER only", COLOR_INNER
    elif inner_seen:
        status, scolor = "INNER only", COLOR_INNER
    else:
        status, scolor = "No target found", (0, 0, 255)
    cv2.putText(
        frame, f"STATUS: {status}", (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, scolor, 2,
    )
    return outer_seen, inner_seen


def run_image(detector, path):
    frame = cv2.imread(path)
    if frame is None:
        print(f"[ERROR] Could not read image: {path}")
        return
    detections = detector.detect(frame)
    outer_seen, inner_seen = draw_detections(frame, detections)
    print(
        f"[RESULT] outer={'yes' if outer_seen else 'no'} "
        f"inner={'yes' if inner_seen else 'no'} "
        f"({len(detections)} layer(s) detected)"
    )
    cv2.imshow("Nested 36h11 (image)", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_webcam(detector, camera_index):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera index {camera_index}.")
        return

    print("[INFO] Webcam started. Aim at a nested board. Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        detections = detector.detect(frame)
        draw_detections(frame, detections)
        cv2.imshow("Nested 36h11 (webcam)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Off-drone nested 36h11 detection (webcam or PNG)."
    )
    parser.add_argument("--image", help="path to a PNG to test instead of the webcam")
    parser.add_argument("--camera", type=int, default=0, help="webcam device index")
    parser.add_argument("--outer-id", type=int, default=OUTER_TAG_ID,
                        help=f"outer tag ID (default {OUTER_TAG_ID})")
    parser.add_argument("--inner-id", type=int, default=INNER_TAG_ID,
                        help=f"inner tag ID (default {INNER_TAG_ID})")
    args = parser.parse_args()

    # No intrinsics -> detection/corners only (pose is not needed for testing).
    detector = NestedArucoDetector(
        outer_id=args.outer_id,
        inner_id=args.inner_id,
    )

    if args.image:
        run_image(detector, args.image)
    else:
        run_webcam(detector, args.camera)


if __name__ == "__main__":
    main()
