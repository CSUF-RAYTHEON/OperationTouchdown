import cv2

import depthai as dai

import numpy as np

from pupil_apriltags import Detector

TAG_SIZE = 0.20

TARGET_TAG_ID = 67

with dai.Device() as device:

    print("[INFO] Camera started")

    # -------- CALIBRATION --------

    calib = device.getCalibration()

    # We'll update intrinsics after first frame (safer)

    camera_matrix = None

    dist_coeffs = None

    FX = FY = CX = CY = None

    # -------- DETECTOR --------

    detector = Detector(

        families="tag36h11",

        nthreads=1,

        quad_decimate=1.0,

        refine_edges=1

    )

    # -------- PIPELINE --------

    with dai.Pipeline(device) as pipeline:

        cam_rgb = pipeline.create(dai.node.Camera).build(

            dai.CameraBoardSocket.CAM_A

        )

        rgb_out = cam_rgb.requestOutput(

            size=(640, 480),   # good balance of speed/accuracy

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

            # -------- FIX: Match intrinsics to actual frame --------

            if camera_matrix is None:

                h, w = frame.shape[:2]

                intrinsics = calib.getCameraIntrinsics(

                    dai.CameraBoardSocket.CAM_A,

                    w,

                    h

                )

                camera_matrix = np.array(intrinsics)

                dist_coeffs = np.array(

                    calib.getDistortionCoefficients(

                        dai.CameraBoardSocket.CAM_A

                    )

                )

                FX = camera_matrix[0][0]

                FY = camera_matrix[1][1]

                CX = camera_matrix[0][2]

                CY = camera_matrix[1][2]

                print(f"[INFO] Intrinsics set for {w}x{h}")

            # -------- PROCESS FRAME --------

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            gray = cv2.undistort(

                gray,

                camera_matrix,

                dist_coeffs

            )

            detections = detector.detect(

                gray,

                estimate_tag_pose=True,

                camera_params=(FX, FY, CX, CY),

                tag_size=TAG_SIZE

            )

            for tag in detections:

                if tag.tag_id != TARGET_TAG_ID:

                    continue

                t = tag.pose_t

                cam_x = float(t[0][0])

                cam_y = float(t[1][0])

                cam_z = float(t[2][0])

                # -------- YOUR CONVERSION --------

                body_x = -cam_y

                body_y = cam_x

                body_z = cam_z

                # -------- DEBUG PRINT --------

                print("CAM :", cam_x, cam_y, cam_z)

                print("BODY:", body_x, body_y, body_z)

                print("DIFF:", body_x - cam_x, body_y - cam_y, body_z - cam_z)

                print("------")

                # -------- DRAW TAG --------

                corners = tag.corners.astype(int)

                for i in range(4):

                    cv2.line(frame,

                             tuple(corners[i]),

                             tuple(corners[(i+1) % 4]),

                             (0, 255, 0), 2)
                center = tuple(tag.center.astype(int))

                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # -------- TEXT --------

                cv2.putText(frame, f"X: {body_x:.2f}", (10, 30),

                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                cv2.putText(frame, f"Y: {body_y:.2f}", (10, 60),

                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                cv2.putText(frame, f"Z: {body_z:.2f}", (10, 90),

                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            cv2.imshow("AprilTag Test (v3)", frame)

            if cv2.waitKey(1) == ord('q'):

                break

    cv2.destroyAllWindows()