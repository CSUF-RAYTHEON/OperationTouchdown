import time
import depthai as dai
from Detectors.april_tag_detector import AprilTagDetector
from PixhawkController.stationary_landing_controller import StationaryLandingController

# Reminder: Make sure this matches what you found via 'ls /dev/tty*'
CONNECTION_STRING = "/dev/serial0"
BAUDRATE = 57600
LANDING_THRESHOLD_Z  = 0.4   # meters — trigger landing when tag is this close below
LANDING_THRESHOLD_XY = 0.2   # meters — lateral alignment tolerance (each axis)
                              # 0.1 m was too tight; proportional control + filter
                              # lag means both axes rarely hit 10 cm simultaneously
TAKEOFF_ALTITUDE = 3  # meters

# Number of consecutive frames that must satisfy the landing conditions
# before the landing sequence is committed to.  A single-frame trigger
# can fire on a momentary noisy pose reading.  3 frames @ 30 fps = 100 ms
# of confirmed alignment — enough to filter noise without delaying response.
LANDING_CONFIRM_FRAMES = 3

# Camera output resolution fed to the AprilTag detector.
# 640×640 gives good tag visibility at 1–4 m altitude without
# overwhelming the preprocessing pipeline on the Pi.
# Lower to (300, 300) if CPU becomes a bottleneck.
CAMERA_RESOLUTION = (640, 640)

# 1. Initialize the Device first
with dai.Device() as device:
    print("[INFO] OAK-D Started")

    # Read calibration before starting the pipeline
    calibration = device.getCalibration()

    april_tag_detector = AprilTagDetector(calibration)
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
        controller.takeoff_to_altitude(TAKEOFF_ALTITUDE)  # 3 meters

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
            in_rgb = q_rgb.tryGet()

            if in_rgb is None:
                time.sleep(0.01)
                continue

            frame = in_rgb.getCvFrame()
            pose = april_tag_detector.get_tag_pose(frame)

            # --- LOGIC: The Fallback State Machine ---
            if pose is None:
                landing_confirm_count = 0  # lost the tag — reset confirmation
                time_lost = time.time() - last_tag_time

                if time_lost < HOVER_TIMEOUT:
                    print(f"[WARN] Tag lost for {time_lost:.1f}s. Hovering patiently...")
                    controller.send_velocity(0, 0, 0)

                elif time_lost < SEARCH_TIMEOUT:
                    print(f"[WARN] Tag lost for {time_lost:.1f}s. Ascending to widen FOV...")
                    controller.send_velocity(0, 0, -0.2)

                else:
                    print("[CRITICAL] Tag lost for 12+ seconds. Initiating blind VIO landing...")
                    controller.send_velocity(0, 0, 0.3)

                continue

            # --- LOGIC: Tag is visible ---
            last_tag_time = time.time()

            cam_x, cam_y, cam_z = pose
            print(f"[INFO] Tag Position (Camera): X={cam_x:.2f}, Y={cam_y:.2f}, Z={cam_z:.2f} m")

            body_x, body_y, body_z = controller.convert_camera_to_body_frame(
                cam_x, cam_y, cam_z
            )

            print(f"[INFO] Tag Position (Body): X={body_x:.2f}, Y={body_y:.2f}, Z={body_z:.2f}")

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

            controller.adjust_velocity_and_send(body_x, body_y, body_z)