import time
import cv2
import numpy as np
import depthai as dai
from pupil_apriltags import Detector
from pymavlink import mavutil

# ─────────────────────────────────────────────────────────────────────────────
# Flight Config
# ─────────────────────────────────────────────────────────────────────────────

CONNECTION_STRING = "/dev/serial0"
BAUDRATE          = 57600
TAKEOFF_ALTITUDE  = 6       # meters

# Landing is committed when the tag is within these tolerances for
# LANDING_CONFIRM_FRAMES consecutive frames.
LANDING_THRESHOLD_XY   = 0.2   # meters, lateral (each axis)
LANDING_THRESHOLD_Z    = 0.4   # meters, altitude above tag
LANDING_CONFIRM_FRAMES = 3

# Fallback timers when the tag is lost mid-flight
HOVER_TIMEOUT  = 7.0    # seconds — hover patiently
SEARCH_TIMEOUT = 10.0   # seconds — ascend to widen FOV; blind land after this

# ─────────────────────────────────────────────────────────────────────────────
# Patrol Config
# ─────────────────────────────────────────────────────────────────────────────

# Velocities are in MAV_FRAME_BODY_NED — all movement is drone-relative, so
# the patrol shape is the same regardless of compass heading at takeoff.
PATROL_SPEED = 0.3  # m/s — keep conservative for first tests

# Each segment: (vx, vy, vz, duration_seconds)
# Default pattern: a square — forward → right → backward → left → back home
PATROL_SEGMENTS = [
    ( PATROL_SPEED,  0.0,          0.0, 3.0),
    ( 0.0,           PATROL_SPEED, 0.0, 3.0),
    (-PATROL_SPEED,  0.0,          0.0, 3.0),
    ( 0.0,          -PATROL_SPEED, 0.0, 3.0),
]

# ─────────────────────────────────────────────────────────────────────────────
# Camera Config
# ─────────────────────────────────────────────────────────────────────────────

# 300×300 matches what is used in the test script and is sufficient for tag
# detection at 1–4 m.  Raise to (640, 640) if detection range needs to extend.
CAMERA_RESOLUTION = (300, 300)

# ─────────────────────────────────────────────────────────────────────────────
# MAVLink constants
# ─────────────────────────────────────────────────────────────────────────────

Kp_xy        = 0.4
Kp_z         = 0.3
MAX_VELOCITY = 0.3

# ─────────────────────────────────────────────────────────────────────────────
# AprilTag detection — inlined from UAV/Detectors/april_tag_detector.py
# ─────────────────────────────────────────────────────────────────────────────

TARGET_TAG_ID = 67
TAG_SIZE      = 0.20    # 20 cm tag

_GAMMA_MODERATE      = 2.2
_GAMMA_DEEP          = 4.0
_GAMMA_MID_DARK      = 6.0
_GAMMA_EXTREME       = 8.0
_GAMMA_WHITE_MILD     = 0.5
_GAMMA_WHITE_MODERATE = 0.3
_GAMMA_WHITE_STRONG   = 0.15
_STRETCH_LOW_PCT      = 1.0
_STRETCH_HIGH_PCT     = 99.0


def _build_gamma_lut(gamma: float) -> np.ndarray:
    inv = 1.0 / gamma
    return np.array(
        [min(int((i / 255.0) ** inv * 255.0 + 0.5), 255) for i in range(256)],
        dtype=np.uint8,
    )


def _percentile_stretch(img: np.ndarray) -> np.ndarray:
    lo = np.percentile(img, _STRETCH_LOW_PCT)
    hi = np.percentile(img, _STRETCH_HIGH_PCT)
    if hi <= lo:
        return img
    return np.clip(
        (img.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255
    ).astype(np.uint8)


def _division_normalize(img: np.ndarray, blur_ksize: int = 71) -> np.ndarray:
    if blur_ksize % 2 == 0:
        blur_ksize += 1
    local_mean = cv2.GaussianBlur(img, (blur_ksize, blur_ksize), 0).astype(np.float32) + 1.0
    return np.clip(img.astype(np.float32) / local_mean * 128.0, 0, 255).astype(np.uint8)


def _unsharp_mask(img: np.ndarray, sigma: float = 2.0, strength: float = 1.5) -> np.ndarray:
    blurred = cv2.GaussianBlur(img, (0, 0), sigmaX=sigma)
    return cv2.addWeighted(img, 1.0 + strength, blurred, -strength, 0)


def _local_std_normalize(img: np.ndarray, ksize: int = 31, scale: float = 48.0) -> np.ndarray:
    img_f = img.astype(np.float32)
    k = (ksize, ksize)
    local_mean    = cv2.GaussianBlur(img_f, k, 0)
    local_sq_mean = cv2.GaussianBlur(img_f * img_f, k, 0)
    local_std     = np.sqrt(np.maximum(local_sq_mean - local_mean * local_mean, 0.0)) + 1.0
    return np.clip((img_f - local_mean) / local_std * scale + 128.0, 0, 255).astype(np.uint8)


def _adaptive_thresh(img: np.ndarray, block_size: int, c: int = 5) -> np.ndarray:
    if block_size % 2 == 0:
        block_size += 1
    return cv2.adaptiveThreshold(
        img, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size,
        c,
    )


class AprilTagDetector:

    def __init__(self, calibration_handler):
        self.calibration_handler = calibration_handler
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.FX = self.FY = self.CX = self.CY = None

        self._clahe      = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self._clahe_deep = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))

        self._gamma_lut              = _build_gamma_lut(_GAMMA_MODERATE)
        self._gamma_lut_strong       = _build_gamma_lut(_GAMMA_DEEP)
        self._gamma_lut_mid_dark     = _build_gamma_lut(_GAMMA_MID_DARK)
        self._gamma_lut_extreme      = _build_gamma_lut(_GAMMA_EXTREME)
        self._gamma_lut_white_mild     = _build_gamma_lut(_GAMMA_WHITE_MILD)
        self._gamma_lut_white_moderate = _build_gamma_lut(_GAMMA_WHITE_MODERATE)
        self._gamma_lut_white_strong   = _build_gamma_lut(_GAMMA_WHITE_STRONG)

        self.detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        print("[INFO] AprilTag detector initialized")

    def _update_intrinsics(self, frame):
        h, w = frame.shape[:2]
        intrinsics = self.calibration_handler.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, w, h
        )
        self.camera_matrix = np.array(intrinsics)
        self.FX = self.camera_matrix[0][0]
        self.FY = self.camera_matrix[1][1]
        self.CX = self.camera_matrix[0][2]
        self.CY = self.camera_matrix[1][2]
        self.dist_coeffs = np.array(
            self.calibration_handler.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        )
        print(f"[INFO] Intrinsics updated for {w}x{h}")

    def _preprocess_variants(self, gray: np.ndarray) -> list:
        # Shadow tier
        p1  = self._clahe.apply(gray)
        p2  = _unsharp_mask(p1)
        p3  = self._clahe.apply(cv2.LUT(gray, self._gamma_lut))
        p4  = self._clahe.apply(cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75))
        div_norm = _division_normalize(gray)
        p5  = self._clahe_deep.apply(div_norm)
        p6  = self._clahe_deep.apply(_unsharp_mask(div_norm))
        lsdn = _local_std_normalize(gray)
        p7  = self._clahe_deep.apply(lsdn)
        p8  = self._clahe_deep.apply(_unsharp_mask(lsdn))
        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        p9  = self._clahe_deep.apply(gamma_strong)
        p10 = self._clahe_deep.apply(cv2.bilateralFilter(gamma_strong, d=9, sigmaColor=75, sigmaSpace=75))
        gamma_mid_dark = cv2.LUT(gray, self._gamma_lut_mid_dark)
        p11 = self._clahe_deep.apply(gamma_mid_dark)
        p12 = self._clahe_deep.apply(_division_normalize(gamma_mid_dark))
        p13 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_extreme))
        p14 = self._clahe_deep.apply(_percentile_stretch(gray))
        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p15 = self._clahe_deep.apply(_percentile_stretch(denoised))
        p16 = _adaptive_thresh(gray, block_size=31)
        p17 = _adaptive_thresh(gray, block_size=71)

        # Mixed / dappled light tier
        p18 = self._clahe_deep.apply(_local_std_normalize(denoised))

        # Glare / over-exposure tier
        p19 = self._clahe.apply(div_norm)
        white_mild = cv2.LUT(gray, self._gamma_lut_white_mild)
        p20 = self._clahe.apply(white_mild)
        p21 = self._clahe.apply(_unsharp_mask(white_mild))
        p22 = self._clahe_deep.apply(_division_normalize(white_mild))
        p23 = self._clahe_deep.apply(_local_std_normalize(white_mild))
        white_moderate = cv2.LUT(gray, self._gamma_lut_white_moderate)
        p24 = self._clahe_deep.apply(white_moderate)
        p25 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_white_strong))
        bright_denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p26 = self._clahe_deep.apply(cv2.LUT(bright_denoised, self._gamma_lut_white_moderate))
        p27 = _adaptive_thresh(white_moderate, block_size=31)

        return [
            p1,  p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9,
            p10, p11, p12, p13, p14, p15, p16, p17,
            p18,
            p19, p20, p21, p22, p23, p24, p25, p26, p27,
        ]

    def _detect_raw(self, image: np.ndarray):
        detections = self.detector.detect(
            image,
            estimate_tag_pose=True,
            camera_params=(self.FX, self.FY, self.CX, self.CY),
            tag_size=TAG_SIZE,
        )
        for tag in detections:
            if tag.tag_id == TARGET_TAG_ID:
                return tag
        return None

    def _preprocess_and_find(self, gray: np.ndarray):
        for variant in self._preprocess_variants(gray):
            tag = self._detect_raw(variant)
            if tag is not None:
                return tag
        return None

    def _prepare_gray(self, frame):
        if self.camera_matrix is None:
            self._update_intrinsics(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.undistort(gray, self.camera_matrix, self.dist_coeffs)

    def get_tag_detection(self, frame):
        """Return the raw Detection object for TARGET_TAG_ID, or None."""
        if frame is None:
            return None
        return self._preprocess_and_find(self._prepare_gray(frame))


# ─────────────────────────────────────────────────────────────────────────────
# Pixhawk controller — inlined from UAV/PixhawkController/stationary_landing_controller.py
# ─────────────────────────────────────────────────────────────────────────────

class FlightController:

    def __init__(self, connection_string, baudrate):
        print("[INFO] Connecting to Pixhawk...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.target_system    = 1
        self.master.target_component = 1
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"[INFO] Pixhawk connected  sys={self.master.source_system}  comp={self.master.source_component}")

        self.prev_x = None
        self.prev_y = None
        self.prev_z = None

    # ── Mode / arming ────────────────────────────────────────────────────────

    def change_flight_mode(self, flight_mode):
        print(f"[INFO] Switching to {flight_mode} mode...")
        start = time.time()
        while time.time() - start < 3:
            self.master.recv_match(blocking=False)
            time.sleep(0.1)
        self.master.wait_heartbeat()
        self.master.set_mode(flight_mode)

        start = time.time()
        while time.time() - start < 3:
            ack = self.master.recv_match(type=["COMMAND_ACK"], blocking=True, timeout=2)
            if ack and ack.command == 176 and ack.result == 0:
                print(f"[INFO] Mode change to {flight_mode} accepted")
                break

        start = time.time()
        while time.time() - start < 3:
            hb = self.master.recv_match(type=["HEARTBEAT"], blocking=True, timeout=2)
            if hb and self.master.flightmode == flight_mode:
                print(f"[INFO] Now in {self.master.flightmode} mode")
                break

    def arm_motors(self):
        print("[INFO] Setting arming parameters...")
        params = {
            "ARMING_REQUIRE": 1,
            "ARMING_CHECK": 1,
            "ARMING_ACCTHRESH": 0.3,
            "ARMING_MAGTHRESH": 75,
            "ARMING_NEED_LOC": 0,
        }
        for name, value in params.items():
            try:
                self.master.mav.param_set_send(
                    self.master.target_system, self.master.target_component,
                    name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                )
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
                print(f"  {name} = {value}  ({msg.to_dict() if msg else 'no ack'})")
            except Exception as e:
                print(f"  Failed to set {name}: {e}")
        self.master.wait_heartbeat()

        print("[INFO] Arming motors...")
        self.master.arducopter_arm()
        start = time.time()
        while time.time() - start < 3:
            ack = self.master.recv_match(type=["COMMAND_ACK"], blocking=True, timeout=2)
            if ack and ack.command == 400 and ack.result == 0:
                print("[INFO] ARM command accepted")
                break
        start = time.time()
        while time.time() - start < 3:
            self.master.motors_armed_wait()
            if self.master.motors_armed():
                print("[INFO] Motors armed")
                break

    def disarm_motors(self):
        print("[INFO] Disarming motors...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        self.master.motors_disarmed_wait()
        print("[INFO] Motors disarmed")

    # ── Takeoff / landing ────────────────────────────────────────────────────

    def takeoff_to_altitude(self, meters):
        print(f"[INFO] Taking off to {meters} m...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, meters,
        )
        start = time.time()
        while time.time() - start < 30:
            msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
            if msg is not None:
                altitude = -msg.z
                print(f"[INFO] Altitude: {altitude:.2f} m / {meters} m")
                if altitude >= meters - 0.3:
                    print(f"[INFO] Target altitude reached ({altitude:.2f} m)")
                    return
            time.sleep(0.1)
        print("[WARN] Altitude timeout — proceeding anyway")

    def land(self):
        """Switch to LAND mode and wait for touchdown + auto-disarm."""
        print("[INFO] Switching to LAND mode...")
        self.change_flight_mode("LAND")
        print("[INFO] Waiting for touchdown and auto-disarm...")
        start = time.time()
        while time.time() - start < 20:
            if not self.master.motors_armed():
                print("[INFO] Motors disarmed — touchdown confirmed")
                return
            time.sleep(0.5)
        print("[WARN] Touchdown not confirmed after 20 s — force-disarming")
        self.disarm_motors()

    # ── Velocity control ─────────────────────────────────────────────────────

    def send_velocity(self, vx, vy, vz):
        """Send a body-frame velocity command."""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )

    @staticmethod
    def camera_to_body(cam_x, cam_y, cam_z):
        """Convert camera frame to body frame (downward-facing camera)."""
        return -cam_y, cam_x, cam_z

    def adjust_velocity_and_send(self, body_x, body_y, body_z):
        """Apply exponential smoothing + proportional control, then send."""
        if self.prev_x is None:
            self.prev_x, self.prev_y, self.prev_z = body_x, body_y, body_z

        alpha = 0.7
        self.prev_x = alpha * self.prev_x + (1 - alpha) * body_x
        self.prev_y = alpha * self.prev_y + (1 - alpha) * body_y
        self.prev_z = alpha * self.prev_z + (1 - alpha) * body_z

        bx, by, bz = self.prev_x, self.prev_y, self.prev_z

        thresh = 0.05
        bx = 0.0 if abs(bx) < thresh else bx
        by = 0.0 if abs(by) < thresh else by

        TARGET_Z  = 0.3
        error_z   = bz - TARGET_Z
        vx = Kp_xy * bx
        vy = Kp_xy * by
        vz = 0.0 if abs(error_z) < 0.05 else Kp_z * error_z

        if bz < 0.5:
            vx *= 0.5
            vy *= 0.5

        vx = max(min(vx, MAX_VELOCITY), -MAX_VELOCITY)
        vy = max(min(vy, MAX_VELOCITY), -MAX_VELOCITY)
        vz = max(min(vz, MAX_VELOCITY), -MAX_VELOCITY)

        self.send_velocity(vx, vy, vz)


# ─────────────────────────────────────────────────────────────────────────────
# Visualization helpers
# ─────────────────────────────────────────────────────────────────────────────

def draw_tag(frame, tag):
    """Outline the tag and mark its center."""
    corners = tag.corners.astype(int)
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
    cv2.circle(frame, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)


def draw_overlay(frame, phase, cam_x, cam_y, cam_z, body_x, body_y, body_z,
                 status, confirm, confirm_needed):
    """Render all telemetry text onto the frame."""
    status_color = (0, 255, 0) if status == "ALIGNED" else (0, 165, 255)

    lines = [
        (f"PHASE   {phase}",                          (255, 255, 0)),
        (f"STATUS  {status}",                         status_color),
        (f"CAM   x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}", (0, 255, 0)),
        (f"BODY  x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}", (0, 255, 0)),
        (f"CONFIRM {confirm}/{confirm_needed}",        (0, 255, 255)),
    ]
    for idx, (text, color) in enumerate(lines):
        cv2.putText(frame, text, (10, 20 + idx * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def draw_no_tag(frame, phase, time_lost):
    """Show a minimal overlay when the tag is not visible."""
    cv2.putText(frame, f"PHASE   {phase}",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
    cv2.putText(frame, f"NO TAG  {time_lost:.1f}s",
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)


# ─────────────────────────────────────────────────────────────────────────────
# Patrol
# ─────────────────────────────────────────────────────────────────────────────

def run_patrol(controller, q_rgb, window_title):
    """
    Fly the patrol pattern while streaming the camera feed to the window.
    The camera runs during the patrol so the user can see when the tag first
    comes into view, and the window stays live throughout.
    """
    print("[INFO] Starting patrol phase...")
    leg_labels = ["forward", "right", "backward", "left"]

    for i, (vx, vy, vz, duration) in enumerate(PATROL_SEGMENTS):
        label = leg_labels[i] if i < len(leg_labels) else f"leg {i}"
        print(f"[INFO] Patrol leg {i + 1}/{len(PATROL_SEGMENTS)}: {label} for {duration}s")
        t_start = time.time()
        while time.time() - t_start < duration:
            # Bail out immediately if the FCU disarmed (failsafe, RC override,
            # unexpected ground contact) so we don't keep streaming velocity
            # commands to a disarmed drone.
            controller.master.recv_match(blocking=False)
            if not controller.master.motors_armed():
                print("[INFO] Motors disarmed mid-patrol — exiting.")
                return True

            controller.send_velocity(vx, vy, vz)

            # Keep the window live; show raw feed during patrol
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                cv2.putText(frame, f"PHASE   PATROL  leg {i + 1}/{len(PATROL_SEGMENTS)}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                cv2.imshow(window_title, frame)
                if cv2.waitKey(1) == ord('q'):
                    return True   # signal early exit

            time.sleep(0.1)

    print("[INFO] Patrol complete — hovering...")
    controller.send_velocity(0, 0, 0)
    time.sleep(1.5)
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

WINDOW_TITLE = "Patrol Landing"

with dai.Device() as device:
    print("[INFO] OAK-D started")
    calibration = device.getCalibration()

    detector   = AprilTagDetector(calibration)
    controller = FlightController(CONNECTION_STRING, BAUDRATE)

    with dai.Pipeline(device) as pipeline:

        cam_rgb = pipeline.create(dai.node.Camera)
        cam_rgb.build(dai.CameraBoardSocket.CAM_A)

        rgb_out = cam_rgb.requestOutput(
            size=CAMERA_RESOLUTION,
            type=dai.ImgFrame.Type.NV12,
            fps=30,
        )
        q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)

        pipeline.start()
        print("[INFO] Pipeline started — initiating takeoff...")

        controller.change_flight_mode("GUIDED")
        controller.arm_motors()
        controller.takeoff_to_altitude(TAKEOFF_ALTITUDE)

        # ── Patrol ────────────────────────────────────────────────────────────
        early_exit = run_patrol(controller, q_rgb, WINDOW_TITLE)
        if early_exit:
            cv2.destroyAllWindows()
            raise SystemExit("Patrol aborted by user")

        # ── Landing detection loop ────────────────────────────────────────────
        print("[INFO] Entering landing detection loop...")
        last_tag_time        = time.time()
        landing_confirm_count = 0

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
            tag   = detector.get_tag_detection(frame)

            if tag is None:
                landing_confirm_count = 0
                time_lost = time.time() - last_tag_time

                if time_lost < HOVER_TIMEOUT:
                    print(f"[WARN] Tag lost for {time_lost:.1f}s — hovering")
                    controller.send_velocity(0, 0, 0)
                elif time_lost < SEARCH_TIMEOUT:
                    print(f"[WARN] Tag lost for {time_lost:.1f}s — ascending to widen FOV")
                    controller.send_velocity(0, 0, -0.2)
                else:
                    # Previously this branch just sent a downward velocity in
                    # GUIDED mode, which would land the drone but leave the
                    # detection loop running indefinitely.  Switching to LAND
                    # mode hands control to ArduCopter's ground-detection so
                    # the script exits cleanly on touchdown.
                    print("[CRITICAL] Tag lost 10+ s — committing to LAND mode")
                    draw_no_tag(frame, "LANDING", time_lost)
                    cv2.imshow(WINDOW_TITLE, frame)
                    cv2.waitKey(1)
                    controller.land()
                    break

                draw_no_tag(frame, "LANDING", time_lost)
                cv2.imshow(WINDOW_TITLE, frame)
                if cv2.waitKey(1) == ord('q'):
                    break
                continue

            # ── Tag visible ───────────────────────────────────────────────────
            last_tag_time = time.time()

            t     = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])
            print(f"[INFO] CAM  x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}")

            body_x, body_y, body_z = FlightController.camera_to_body(cam_x, cam_y, cam_z)
            print(f"[INFO] BODY x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}")

            conditions_met = (
                abs(body_x) < LANDING_THRESHOLD_XY and
                abs(body_y) < LANDING_THRESHOLD_XY and
                body_z < LANDING_THRESHOLD_Z
            )

            if conditions_met:
                landing_confirm_count += 1
                print(f"[INFO] Landing condition met "
                      f"({landing_confirm_count}/{LANDING_CONFIRM_FRAMES}) — "
                      f"x={body_x:.2f} y={body_y:.2f} z={body_z:.2f}")
                if landing_confirm_count >= LANDING_CONFIRM_FRAMES:
                    print("[INFO] Landing confirmed — executing landing sequence")
                    draw_tag(frame, tag)
                    draw_overlay(frame, "LANDING", cam_x, cam_y, cam_z,
                                 body_x, body_y, body_z,
                                 "ALIGNED", landing_confirm_count, LANDING_CONFIRM_FRAMES)
                    cv2.imshow(WINDOW_TITLE, frame)
                    cv2.waitKey(1)
                    controller.land()
                    break
            else:
                landing_confirm_count = 0

            controller.adjust_velocity_and_send(body_x, body_y, body_z)

            # ── Visualize ─────────────────────────────────────────────────────
            draw_tag(frame, tag)
            status = "ALIGNED" if conditions_met else "ADJUSTING"
            draw_overlay(frame, "LANDING", cam_x, cam_y, cam_z,
                         body_x, body_y, body_z,
                         status, landing_confirm_count, LANDING_CONFIRM_FRAMES)
            cv2.imshow(WINDOW_TITLE, frame)
            if cv2.waitKey(1) == ord('q'):
                break

cv2.destroyAllWindows()
