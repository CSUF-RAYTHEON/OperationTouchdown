import time
import depthai as dai

from Detectors.april_tag_detector import AprilTagDetector
from PixhawkController.stationary_landing_controller import (
    StationaryLandingController
)

CONNECTION_STRING = "/dev/serial0"
BAUDRATE = 57600

LANDING_THRESHOLD = 0.3
TAKEOFF_ALTITUDE = 5  # meters

HOVER_TIMEOUT = 7.0
SEARCH_TIMEOUT = 10.0

# Start the pipeline / camera

with dai.Device() as device:

    print("[INFO] OAK-D Started")

    # Load calibration for AprilTag detection
    calibration = device.getCalibration()

    april_tag_detector = AprilTagDetector(calibration)

    controller = StationaryLandingController(
        CONNECTION_STRING,
        BAUDRATE
    )

    # Create the pipeline

    with dai.Pipeline(device) as pipeline:
        cam_rgb = pipeline.create(dai.node.Camera)
        cam_rgb.build(dai.CameraBoardSocket.CAM_A)

        rgb_out = cam_rgb.requestOutput(
            size=(640, 480),
            type=dai.ImgFrame.Type.BGR888p,
            fps=30
        )

        q_rgb = rgb_out.createOutputQueue(
            maxSize=4,
            blocking=False
        )

        pipeline.start()

        print("[INFO] Pipeline started")
        print("[INFO] Initiating takeoff sequence...")

        controller.change_flight_mode("GUIDED")
        controller.arm_motors()
        controller.takeoff_to_altitude(TAKEOFF_ALTITUDE)

        last_tag_time = time.time()
        
        # Track the last known positions for the blind-spot check
        last_body_x = 99.0
        last_body_y = 99.0
        last_body_z = 99.0

        while pipeline.isRunning():
            in_rgb = q_rgb.tryGet()

            if in_rgb is None:
                time.sleep(0.01)
                continue
            
            # Get the current frame
            frame = in_rgb.getCvFrame()

            # Get the x y z from the april tag
            pose = april_tag_detector.get_tag_pose(frame)

                # --- BLIND SPOT OVERRIDE WITH GUIDED DESCENT ---
            if pose is None:
                if last_body_z < 0.5 and abs(last_body_x) < 0.20 and abs(last_body_y) < 0.20:
                    print("[INFO] Tag lost in blind spot. Executing Active Guided Descent.")
                    
                    # Push straight down in GUIDED mode to avoid EKF drift
                    descent_start = time.time()
                    while time.time() - descent_start < 2.5: # 2.5 seconds to reach the floor
                        controller.send_velocity(0, 0, 0.25) # 0.25 m/s downwards
                        time.sleep(0.1)
                        
                    print("[INFO] Touchdown assumed. Disarming motors.")
                    controller.disarm_motors()
                    break
                # ... (rest of the blind spot hovering logic stays the same)

                time_lost = time.time() - last_tag_time

                if time_lost < HOVER_TIMEOUT:
                    print(f"[WARN] Tag lost for {time_lost:.1f}s. Hovering...")
                    controller.send_velocity(0, 0, 0)

                elif time_lost < SEARCH_TIMEOUT:
                    print(f"[WARN] Tag lost for {time_lost:.1f}s. Ascending...")
                    controller.send_velocity(0, 0, -0.2) # Negative Z is UP

                else:
                    print("[CRITICAL] Tag lost too long. Blind landing...")
                    controller.stationary_landing()
                    time.sleep(5)
                    controller.disarm_motors()
                    break

                continue

            # tag detected, reset timer
            last_tag_time = time.time()
            cam_x, cam_y, cam_z = pose
            
            # camera to body frame conversion
            body_x, body_y, body_z = controller.convert_camera_to_body_frame(cam_x, cam_y, cam_z)
            
            # Update our memory of where we are
            last_body_x = body_x
            last_body_y = body_y
            last_body_z = body_z

            print(f"[INFO] Body Frame | X={body_x:.2f}, Y={body_y:.2f}, Z={body_z:.2f}")

            # --- NORMAL LANDING TRIGGER WITH GUIDED DESCENT ---
            if abs(body_x) < 0.15 and abs(body_y) < 0.15 and body_z < 0.4:
                print("[INFO] Landing conditions reached. Executing Active Guided Descent.")
                
                # Push straight down in GUIDED mode to avoid EKF drift
                descent_start = time.time()
                while time.time() - descent_start < 2.5:
                    controller.send_velocity(0, 0, 0.25) # 0.25 m/s downwards
                    time.sleep(0.1)
                    
                print("[INFO] Touchdown assumed. Disarming motors.")
                controller.disarm_motors()
                break

            # track the tag by sending velocity commands
            controller.adjust_velocity_and_send(body_x, body_y, body_z)
            
            # Small sleep to stabilize loop timing
            time.sleep(0.01)