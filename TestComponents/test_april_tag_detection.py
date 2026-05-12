import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2
import depthai as dai
from UAV.Detectors.april_tag_detector import AprilTagDetector

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

            result = detector.get_tag_pose(frame)

            if result is not None:
                cam_x, cam_y, cam_z = result

                body_x = -cam_y
                body_y = cam_x
                body_z = cam_z

                print(f"CAM:  x={cam_x:.2f}, y={cam_y:.2f}, z={cam_z:.2f}")
                print(f"BODY: x={body_x:.2f}, y={body_y:.2f}, z={body_z:.2f}")
                print("------")

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

            cv2.imshow("AprilTag Test", frame)

            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()