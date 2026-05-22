import time
import cv2
import cv2.aruco as aruco
import depthai as dai
from Detectors.aruco_marker_detector import ArucoMarkerDetector
from PixhawkController.stationary_landing_controller import StationaryLandingController

# Reminder: Make sure this matches what you found via 'ls /dev/tty*'
CONNECTION_STRING = "/dev/serial0"
BAUDRATE = 57600
LANDING_THRESHOLD_Z  = 0.4   # meters — trigger landing when marker is this close below
LANDING_THRESHOLD_XY = 0.2   # meters — lateral alignment tolerance (each axis)
                              # 0.1 m was too tight; proportional control + filter
                              # lag means both axes rarely hit 10 cm simultaneously
TAKEOFF_ALTITUDE = 6  # meters

# Number of consecutive frames that must satisfy the landing conditions
# before the landing sequence is committed to.  A single-frame trigger
# can fire on a momentary noisy pose reading.  3 frames @ 30 fps = 100 ms
# of confirmed alignment — enough to filter noise without delaying response.
LANDING_CONFIRM_FRAMES = 3

# Camera output resolution fed to the ArUco detector.
# 640×640 gives good marker visibility at 1–4 m altitude without
# overwhelming the preprocessing pipeline on the Pi.
# Lower to (300, 300) if CPU becomes a bottleneck.
CAMERA_RESOLUTION = (640, 640)

# 1. Initialize the Device first
with dai.Device() as device:
    print("[INFO] OAK-D Started")

    # Read calibration before starting the pipeline
    calibration = device.getCalibration()

    # Same detector class as stationary_landing_aruco.py, but configured to
    # use the AprilTag 36h11 family via the cv2.aruco backend.
    aruco_marker_detector = ArucoMarkerDetector(
        calibration, dict_id=aruco.DICT_APRILTAG_36h11
    )
    controller = StationaryLandingController(CONNECTION_STRING, BAUDRATE)

    # 2. Create the Pipeline bound to the device
    with dai.Pipeline(device) as pipeline:

        # 3. Create and build the Camera Node (DepthAI v3 API)
        cam_rgb = pipeline.create(dai.node.Camera)
        cam_rgb.build(dai.CameraBoardSocket.CAM_A)

        # 4. Request a scaled output — no XLinkOut node needed
        rgb_out = cam_rgb.requestOutput(
            size=CAMERA_RESOLUTION,
            type=dai.ImgFrame.Type.NV12,
            fps=30
        )

        # 5. Create the output queue directly from the requested output
        q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)

        # 6. Start the pipeline
        pipeline.start()
        print("[INFO] Pipeline started. Initiating takeoff sequence...")

        controller.change_flight_mode("GUIDED")
        controller.arm_motors()
        controller.takeoff_to_altitude(TAKEOFF_ALTITUDE)

        # --- SETUP: The Fallback Timers ---
        last_tag_time = time.time()

        # Counts consecutive frames where all landing conditions are satisfied.
        # Resets to 0 whenever any condition is not met.
        landing_confirm_count = 0

        # Extended Fallback thresholds (in seconds)
        HOVER_TIMEOUT = 7.0   # Patient hover duration
        SEARCH_TIMEOUT = 10.0 # Total time before forcing a blind landing

        # 7. Safe loop checking if the pipeline is still active
        while pipeline.isRunning():
            # Drain MAVLink messages and exit cleanly the instant the FCU
            # reports disarm — auto-disarm on touchdown, RC failsafe, or any
            # ground-detection by ArduCopter.  Without this the loop would
            # keep processing camera frames after the drone is on the ground.
            controller.master.recv_match(blocking=False)
            if not controller.master.motors_armed():
                print("[INFO] Motors disarmed — touchdown detected. Exiting.")
                break

            in_rgb = q_rgb.tryGet()

            if in_rgb is None:
                time.sleep(0.01)
                continue

            frame = in_rgb.getCvFrame()
            tag = aruco_marker_detector.get_tag_detection(frame)

            # --- LOGIC: The Fallback State Machine ---
            if tag is None:
                landing_confirm_count = 0  # lost the marker — reset confirmation
                time_lost = time.time() - last_tag_time

                if time_lost < HOVER_TIMEOUT:
                    print(f"[WARN] Marker lost for {time_lost:.1f}s. Hovering patiently...")
                    controller.send_velocity(0, 0, 0)

                elif time_lost < SEARCH_TIMEOUT:
                    print(f"[WARN] Marker lost for {time_lost:.1f}s. Ascending to widen FOV...")
                    controller.send_velocity(0, 0, -0.2)

                else:
                    # Previously this branch just sent a downward velocity in
                    # GUIDED mode, which would land the drone but leave the
                    # detection loop running indefinitely.  Switching to LAND
                    # mode hands control to ArduCopter's ground-detection so
                    # the script exits cleanly on touchdown.
                    print("[CRITICAL] Marker lost for 12+ seconds. Committing to LAND mode...")
                    controller.stationary_landing()
                    break

                cv2.imshow("Stationary Landing 36h11", frame)
                if cv2.waitKey(1) == ord('q'):
                    break
                continue

            # --- LOGIC: Marker is visible ---
            last_tag_time = time.time()

            t = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])

            print(f"[INFO] Marker Position (Camera): X={cam_x:.2f}, Y={cam_y:.2f}, Z={cam_z:.2f} m")

            body_x, body_y, body_z = controller.convert_camera_to_body_frame(
                cam_x, cam_y, cam_z
            )

            print(f"[INFO] Marker Position (Body): X={body_x:.2f}, Y={body_y:.2f}, Z={body_z:.2f}")

            # --- LANDING CONDITION CHECK ---
            conditions_met = (
                abs(body_x) < LANDING_THRESHOLD_XY and
                abs(body_y) < LANDING_THRESHOLD_XY and
                body_z < LANDING_THRESHOLD_Z
            )

            if conditions_met:
                landing_confirm_count += 1
                print(f"[INFO] Landing condition confirmed "
                      f"({landing_confirm_count}/{LANDING_CONFIRM_FRAMES}) — "
                      f"X={body_x:.2f} Y={body_y:.2f} Z={body_z:.2f}")

                if landing_confirm_count >= LANDING_CONFIRM_FRAMES:
                    print("[INFO] Landing confirmed. Executing landing sequence...")
                    controller.stationary_landing()
                    break
            else:
                # Any frame that misses the condition resets the counter so
                # we never accidentally latch onto a partial run of noisy frames.
                landing_confirm_count = 0

            vx_cmd, vy_cmd, vz_cmd = controller.adjust_velocity_and_send(
                body_x, body_y, body_z
            )

            # --- VISUALIZATION ---
            # Runs after velocity is sent so the flight control path is never
            # delayed by the display call.
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame,
                         tuple(corners[i]),
                         tuple(corners[(i + 1) % 4]),
                         (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            status = "ALIGNED" if conditions_met else "ADJUSTING"
            status_color = (0, 255, 0) if conditions_met else (0, 165, 255)
            cv2.putText(frame, status,
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)
            cv2.putText(frame,
                        f"CAM  x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}",
                        (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame,
                        f"BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}",
                        (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame,
                        f"CMD  vx={vx_cmd:+.2f} vy={vy_cmd:+.2f} vz={vz_cmd:+.2f}",
                        (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
            cv2.putText(frame,
                        f"CONFIRM {landing_confirm_count}/{LANDING_CONFIRM_FRAMES}",
                        (10, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            cv2.imshow("Stationary Landing 36h11", frame)
            if cv2.waitKey(1) == ord('q'):
                break

cv2.destroyAllWindows()
