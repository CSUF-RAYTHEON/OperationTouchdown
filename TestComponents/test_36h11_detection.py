import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2
import cv2.aruco as aruco
import depthai as dai
from UAV.Detectors.aruco_marker_detector import ArucoMarkerDetector

with dai.Device() as device:
    print("[INFO] Camera started")

    calib = device.getCalibration()
    detector = ArucoMarkerDetector(calib, dict_id=aruco.DICT_APRILTAG_36h11)

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

                # draw marker outline
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

            cv2.imshow("36h11 Test", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
