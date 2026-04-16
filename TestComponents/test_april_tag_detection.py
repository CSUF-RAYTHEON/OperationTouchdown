#######################################
# Tests the camera frame to body frame
# conversion and AprilTag detection.
#
# If you move the drone forward, body x should increase.
# If you move the drone to the right, body y should increase.
# If you move the drone down, body z should decrease.
####################################### 
import cv2
import depthai as dai
import numpy as np
from pupil_apriltags import Detector

# -------- CONFIG --------
TAG_SIZE = 0.20
TARGET_TAG_ID = 67

# -------- PIPELINE --------
pipeline = dai.Pipeline()

cam = pipeline.createColorCamera()
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setInterleaved(False)

xout = pipeline.createXLinkOut()
xout.setStreamName("rgb")
cam.video.link(xout.input)

# -------- START DEVICE --------
with dai.Device(pipeline) as device:

    print("[INFO] Camera started")

    q_rgb = device.getOutputQueue("rgb")

    # Calibration
    calib = device.readCalibration()
    intrinsics = calib.getCameraIntrinsics(
        dai.CameraBoardSocket.RGB,
        1920,
        1080
    )

    camera_matrix = np.array(intrinsics)
    dist_coeffs = np.array(
        calib.getDistortionCoefficients(dai.CameraBoardSocket.RGB)
    )

    FX = camera_matrix[0][0]
    FY = camera_matrix[1][1]
    CX = camera_matrix[0][2]
    CY = camera_matrix[1][2]

    # AprilTag detector
    detector = Detector(families="tag36h11")

    while True:
        frame = q_rgb.get().getCvFrame()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.undistort(gray, camera_matrix, dist_coeffs)

        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(FX, FY, CX, CY),
            tag_size=TAG_SIZE
        )

        for tag in detections:
            if tag.tag_id != TARGET_TAG_ID:
                continue

            # -------- CAMERA FRAME --------
            t = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])

            # -------- YOUR CONVERSION --------
            body_x = -cam_y
            body_y = cam_x
            body_z = cam_z

            # -------- PRINT --------
            print(f"CAM:  x={cam_x:.2f}, y={cam_y:.2f}, z={cam_z:.2f}")
            print(f"BODY: x={body_x:.2f}, y={body_y:.2f}, z={body_z:.2f}")
            print("------")

            # -------- DRAW TAG --------
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame,
                         tuple(corners[i]),
                         tuple(corners[(i+1)%4]),
                         (0,255,0), 2)

            center = tuple(tag.center.astype(int))
            cv2.circle(frame, center, 5, (0,0,255), -1)

            # -------- DRAW TEXT --------
            cv2.putText(frame, f"BODY X: {body_x:.2f}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(frame, f"BODY Y: {body_y:.2f}", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(frame, f"BODY Z: {body_z:.2f}", (10,90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("AprilTag Test", frame)

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()