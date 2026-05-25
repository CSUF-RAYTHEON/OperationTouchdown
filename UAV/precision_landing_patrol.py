"""
═══════════════════════════════════════════════════════════════════════════════
 precision_landing_patrol.py
═══════════════════════════════════════════════════════════════════════════════

 What this script does
 ─────────────────────
 Single-file, inline-everything UAV mission that performs an AprilTag-based
 PRECISION LANDING using the autopilot's built-in PrecLand controller.  The
 hand-rolled PD velocity loop used in stationary_landing.py / patrol_landing.py
 is replaced by streaming MAVLink ``LANDING_TARGET`` messages to the FCU; the
 actual descent and corner-of-tag tracking are handled by ArduCopter itself.

 Phase machine
 ─────────────
   INIT → STABILIZE → PATROL ─tag─▶ TRACK (10 s) ─ready─▶ PRECISION_LAND
                                       │                       │
                                       │ tag lost              │ tag lost
                                       ▼                       ▼
                                     SEARCH ◀────tag lost──── (low-alt? → LAND)
                                       │
                                       │ re-patrol acquires tag
                                       ▼
                                     TRACK → PRECISION_LAND → TOUCHDOWN

   SEARCH is bounded by MAX_RESEARCH_ATTEMPTS; on exhaustion the script
   commits to a plain LAND descent at the current position.

 Key MAVLink messages used
 ─────────────────────────
   • COMMAND_LONG → MAV_CMD_NAV_TAKEOFF      .. initial climb to TAKEOFF_ALTITUDE
   • SET_POSITION_TARGET_LOCAL_NED (vel)     .. body-frame hover during STABILIZE,
                                                box-patrol velocity legs
   • SET_POSITION_TARGET_LOCAL_NED (pos)     .. goto_ned() for re-centering above
                                                last known marker position
   • LOCAL_POSITION_NED                      .. ascent monitoring + drift checks
                                                + drone NED used to anchor the
                                                LANDING_TARGET we publish
   • LANDING_TARGET (MAV_FRAME_BODY_FRD)     .. published at ~10 Hz with the
                                                angular form (angle_x, angle_y,
                                                distance) plus the position form
                                                (x/y/z + position_valid=1) so
                                                ArduCopter ≥ 4.1 can run its own
                                                PrecLand descent in LAND mode.
                                                Body-frame is mandatory — the AC
                                                companion driver silently ignores
                                                LOCAL_NED position payloads.
   • PARAM_SET / PARAM_VALUE                 .. PLND_* enable + ARMING_* unlock

 DepthAI v3 pipeline used
 ────────────────────────
   dai.Device  →  dai.Pipeline  →  dai.node.Camera (CAM_A) → requestOutput(...)
                              →  output queue (maxSize=4, blocking=False)

 References
 ──────────
   • PX4 precision-landing docs — conceptual basis for the
     approach → descent → final-approach phasing and search-on-loss behaviour.
   • ArduPilot PrecLand (the actual implementation target on this airframe):
     PLND_ENABLED / PLND_TYPE=1 (companion) / PLND_EST_TYPE / PLND_YAW_ALIGN.

 Hardware target
 ───────────────
   • OAK-D S2 (DepthAI v3 API, NV12 frames at CAMERA_RESOLUTION, 30 fps).
   • Pixhawk running ArduCopter, MAVLink over /dev/serial0 @ 57600 baud.
═══════════════════════════════════════════════════════════════════════════════
"""

import math
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
TAKEOFF_ALTITUDE  = 6           # meters — matches stationary_landing.py

# ─────────────────────────────────────────────────────────────────────────────
# Stabilization Config (NEW — not present in stationary_landing.py /
# patrol_landing.py).  Before we start the patrol we verify the autopilot is
# actually holding altitude and not drifting laterally.  The values that come
# OUT of stabilization are used as the (x_home, y_home) anchor for the patrol.
# ─────────────────────────────────────────────────────────────────────────────

STABILIZE_ALT_TOLERANCE   = 0.3   # m       — ±0.3 m around TAKEOFF_ALTITUDE
STABILIZE_DRIFT_TOLERANCE = 0.5   # m       — horizontal drift from origin
STABILIZE_VEL_TOLERANCE   = 0.3   # m/s     — max horizontal velocity
STABILIZE_HOLD_SECONDS    = 3.0   # how long all three checks must hold true
STABILIZE_TIMEOUT_SECONDS = 10.0  # total time budget; print warning & proceed

# ─────────────────────────────────────────────────────────────────────────────
# Patrol Config
# 5 feet per leg ≈ 1.524 m.  Velocity is BODY-frame so the box rotates with
# the drone's heading at takeoff — no compass dependency.  Pattern order:
#     forward → right → back → left  (returns to origin).
# ─────────────────────────────────────────────────────────────────────────────

BOX_LEG_METERS  = 1.524                       # 5 ft per leg
PATROL_SPEED    = 0.3                         # m/s — keep conservative
LEG_DURATION    = BOX_LEG_METERS / PATROL_SPEED   # ≈ 5.08 s

# Each segment: (vx, vy, vz, duration_s, label).
PATROL_SEGMENTS = [
    ( PATROL_SPEED,  0.0,          0.0, LEG_DURATION, "forward"),
    ( 0.0,           PATROL_SPEED, 0.0, LEG_DURATION, "right"),
    (-PATROL_SPEED,  0.0,          0.0, LEG_DURATION, "back"),
    ( 0.0,          -PATROL_SPEED, 0.0, LEG_DURATION, "left"),
]

# ─────────────────────────────────────────────────────────────────────────────
# Precision-landing Config
# ─────────────────────────────────────────────────────────────────────────────

LANDING_TARGET_RATE_HZ = 10.0     # cap the LANDING_TARGET send rate; the
                                  # autopilot only needs a fresh pose every
                                  # ~100 ms.  More than that just floods the
                                  # serial link.
LANDING_TARGET_MIN_DT  = 1.0 / LANDING_TARGET_RATE_HZ

TAG_LOSS_TIMEOUT       = 3.0      # s — if tag stays missing this long during
                                  # PRECISION_LAND we bail out, climb back,
                                  # and re-fly the box at the last known spot.

MAX_RESEARCH_ATTEMPTS  = 3        # exhaust then fall back to plain LAND mode

GOTO_TOLERANCE         = 0.5      # m — goto_ned() accept radius
GOTO_TIMEOUT           = 20.0     # s — goto_ned() blocking upper bound

TAG_TOO_CLOSE_ALT_M    = 1.5      # if tag is lost below this RELATIVE altitude
                                  # during PRECISION_LAND, commit to plain LAND
                                  # descent rather than re-searching.  At this
                                  # height the tag is almost certainly out of
                                  # the FOV simply because we are nearly on top
                                  # of it; ArduCopter will continue descending
                                  # in LAND mode and auto-disarm on touchdown.

# ─────────────────────────────────────────────────────────────────────────────
# TRACK Phase Config — runs between PATROL and PRECISION_LAND.  After the
# patrol acquires the tag, the drone first holds station above the tag
# using a velocity-PD loop for TRACK_DURATION_S seconds before handing
# control to the autopilot's PrecLand controller.  This guarantees the
# autopilot starts the descent with a centered tag in view; in the old
# flow the very first LANDING_TARGET was sent from a marginal angle and
# the descent began before the lateral loop had a chance to converge.
# ─────────────────────────────────────────────────────────────────────────────

TRACK_DURATION_S       = 10.0     # s — total time spent in TRACK
TRACK_LOSS_TIMEOUT_S   = 3.0      # s — bail to SEARCH after this much loss
TRACK_Kp_XY            = 0.35     # P-gain on body-frame position error
TRACK_Kd_XY            = 0.25     # D-gain on body-frame position error
TRACK_MAX_V_XY         = 0.3      # m/s — per-axis horizontal cap
TRACK_VZ_HOLD          = 0.0      # vertical hold; LAND-relative descent
                                  # comes only after TRACK completes.

# ─────────────────────────────────────────────────────────────────────────────
# Camera Config
# 640×640 (same as stationary_landing.py) gives better detection range than
# patrol_landing.py's 300×300 — important because the autopilot will start
# descending the moment we hand it over, and any tag-loss during descent
# triggers an expensive search.
# ─────────────────────────────────────────────────────────────────────────────

CAMERA_RESOLUTION = (640, 640)
WINDOW_TITLE      = "Precision Landing Patrol"

# ─────────────────────────────────────────────────────────────────────────────
# AprilTag detection — inlined from UAV/Detectors/april_tag_detector.py.
# Kept verbatim per the user's "inlined everything" / single-file requirement.
# ─────────────────────────────────────────────────────────────────────────────

TARGET_TAG_ID = 67
TAG_SIZE      = 0.20    # 20 cm tag

_GAMMA_MODERATE       = 2.2
_GAMMA_DEEP           = 4.0
_GAMMA_MID_DARK       = 6.0
_GAMMA_EXTREME        = 8.0
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
    """Inlined copy of UAV/Detectors/april_tag_detector.py.

    The duplication is deliberate — the user wants a single, self-contained
    script that matches the style of patrol_landing.py.  Modifying the shared
    Detectors/ module is explicitly out-of-scope for this file.
    """

    def __init__(self, calibration_handler):
        self.calibration_handler = calibration_handler
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.FX = self.FY = self.CX = self.CY = None

        self._clahe      = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self._clahe_deep = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))

        self._gamma_lut                = _build_gamma_lut(_GAMMA_MODERATE)
        self._gamma_lut_strong         = _build_gamma_lut(_GAMMA_DEEP)
        self._gamma_lut_mid_dark       = _build_gamma_lut(_GAMMA_MID_DARK)
        self._gamma_lut_extreme        = _build_gamma_lut(_GAMMA_EXTREME)
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
        p18 = self._clahe_deep.apply(_local_std_normalize(denoised))
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
        if frame is None:
            return None
        return self._preprocess_and_find(self._prepare_gray(frame))


# ─────────────────────────────────────────────────────────────────────────────
# Pixhawk controller — extends the inlined-style FlightController from
# patrol_landing.py with everything precision-landing-specific.
# ─────────────────────────────────────────────────────────────────────────────

# SET_POSITION_TARGET_LOCAL_NED typemask bits (1 = IGNORE):
#   bit  0  x  (position)
#   bit  1  y  (position)
#   bit  2  z  (position)
#   bit  3  vx (velocity)
#   bit  4  vy (velocity)
#   bit  5  vz (velocity)
#   bit  6  ax (acceleration)
#   bit  7  ay (acceleration)
#   bit  8  az (acceleration)
#   bit  9  force/acc-is-force
#   bit 10  yaw
#   bit 11  yaw_rate
#
# Velocity-only (matches patrol_landing.py):
#     ignore pos(0,1,2) + acc(6,7,8) + force(9) + yaw(10) + yaw_rate(11)
#     = 0b0000_1111_1100_0111  = 4039
TYPEMASK_VELOCITY_ONLY = 0b0000111111000111
#
# Position-only (used by goto_ned):
#     ignore vel(3,4,5) + acc(6,7,8) + force(9) + yaw(10) + yaw_rate(11)
#     = 0b0000_1111_1111_1000  = 4088
TYPEMASK_POSITION_ONLY = 0b0000111111111000


class PrecisionLandingController:
    """Self-contained MAVLink controller for the precision-landing patrol.

    This mirrors the FlightController class inlined in patrol_landing.py and
    adds the autopilot-driven PrecLand pieces:

      • enable_precland_params()  — set PLND_* at startup
      • send_landing_target()     — publish a LANDING_TARGET frame
      • get_local_position()      — cached LOCAL_POSITION_NED snapshot
      • wait_stabilized()         — post-takeoff hold verification
      • goto_ned()                — position-target goto helper
    """

    # ── Construction / connection ────────────────────────────────────────────

    def __init__(self, connection_string, baudrate):
        print("[INFO] Connecting to Pixhawk...")
        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.target_system    = 1
        self.master.target_component = 1
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"[INFO] Pixhawk connected  sys={self.master.source_system}  "
              f"comp={self.master.source_component}")

        # Cached telemetry — refreshed by _drain_messages() on every loop tick.
        # Initialised to None so callers can detect "no telemetry yet" instead
        # of trusting a stale zero.
        self.last_pos = {
            "x": None, "y": None, "z": None,
            "vx": None, "vy": None, "vz": None,
            "t": 0.0,
        }

        # Last commanded velocity — kept here so draw_overlay() can show
        # whichever motor-driving signal is currently active.  We don't have
        # per-motor PWM telemetry on this airframe, so the velocity/throttle
        # command we publish is the closest stand-in for "which motors are
        # being used" (see the user's overlay spec).
        self.last_cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0,
                         "lt_x": None, "lt_y": None, "lt_z": None}

        # Rate-limit gate for LANDING_TARGET so the serial link is not flooded.
        self._last_lt_send = 0.0

        # TRACK-phase prev state — same EMA + PD pattern as
        # adjust_velocity_and_send() in stationary_landing_controller.py.
        # Re-seeded if (now - prev_t) > 0.5 s so a stale buffer (e.g.
        # after a tag dropout) does not produce a D-term spike on the
        # next valid frame.
        self._track_prev_x = None
        self._track_prev_y = None
        self._track_prev_t = None

        # ── Takeoff anchor ──────────────────────────────────────────────────────
        # LOCAL_POSITION_NED.z is reported relative to the EKF/local origin, NOT
        # the physical takeoff point.  On airframes with a stale GPS/EKF the
        # origin can be hundreds of metres off — we saw a real-flight log where
        # the FIRST LOCAL_POSITION_NED after MAV_CMD_NAV_TAKEOFF reported
        # -z = 339.88 m while the drone was still on the ground.  We therefore
        # cache the z at the moment NAV_TAKEOFF is sent and report altitude
        # RELATIVE TO THAT BASELINE everywhere downstream (takeoff, stabilize,
        # HUD overlay).  None until takeoff_to_altitude() seeds it.
        self.takeoff_z_origin = None

    # ── Telemetry plumbing ───────────────────────────────────────────────────

    def request_telemetry_streams(self):
        """Ask the FCU to stream LOCAL_POSITION_NED at 10 Hz.

        ArduCopter typically sends it by default, but on some configurations
        it does not — the takeoff and stabilization checks both rely on it,
        and a missing stream silently breaks them.  Mirrors the call the
        existing TestComponents/test_*_local_position.py scripts make.
        """
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                1e6 / 10,    # interval in microseconds → 10 Hz
                0, 0, 0, 0, 0,
            )
            print("[INFO] Requested LOCAL_POSITION_NED @ 10 Hz")
        except Exception as e:
            print(f"[WARN] Could not request LOCAL_POSITION_NED stream: {e}")

    def _drain_messages(self):
        """Drain the MAVLink buffer non-blockingly, caching latest telemetry.

        Must be called inside any hot loop — without it the serial link
        eventually overflows and the FCU starts dropping inbound commands.
        The existing scripts call recv_match(blocking=False) for the same
        reason; this just centralises it and stashes useful fields.
        """
        for _ in range(20):   # cap per-tick drain so we never spin forever
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_type() == "LOCAL_POSITION_NED":
                self.last_pos["x"]  = msg.x
                self.last_pos["y"]  = msg.y
                self.last_pos["z"]  = msg.z
                self.last_pos["vx"] = msg.vx
                self.last_pos["vy"] = msg.vy
                self.last_pos["vz"] = msg.vz
                self.last_pos["t"]  = time.time()

    def get_local_position(self):
        """Return the latest (x, y, z, vx, vy, vz) NED snapshot.

        Returns a tuple of Nones when no LOCAL_POSITION_NED has been
        received yet — callers must guard against that.
        """
        self._drain_messages()
        p = self.last_pos
        return p["x"], p["y"], p["z"], p["vx"], p["vy"], p["vz"]

    # ── Mode / arming ────────────────────────────────────────────────────────

    def change_flight_mode(self, flight_mode):
        """Switch flight mode and confirm via COMMAND_ACK + HEARTBEAT."""
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
        """Set arming params, arducopter_arm(), wait for motors_armed."""
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
                    name.encode(), float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32,
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

    # ── ArduCopter PrecLand parameter setup ──────────────────────────────────
    #
    # FIRMWARE REQUIREMENT
    # ────────────────────
    # The position-form LANDING_TARGET payload (``MAV_FRAME_BODY_FRD`` +
    # x/y/z + ``position_valid=1``) we publish from
    # ``send_landing_target()`` only takes effect on ArduCopter ≥ 4.1.
    # Older builds will silently ignore the position fields and use only
    # the angular form (``angle_x``, ``angle_y``, ``distance``) — which
    # is also correctly populated in our payload, so older airframes
    # still get a usable target, just without the absolute-pose hint.
    # If a flight log shows the autopilot descending straight down
    # despite a clearly off-center tag, check the firmware version.

    def enable_precland_params(self):
        """Enable ArduCopter's built-in PrecLand controller.

        Parameters set:
          • PLND_ENABLED   = 1   Enable PrecLand at all.  Without this the
                                 autopilot ignores LANDING_TARGET messages.
          • PLND_TYPE      = 1   "Companion" — tells AP that pose updates
                                 will arrive over MAVLink instead of from an
                                 IRLock sensor wired to the FCU.
          • PLND_EST_TYPE  = 0   "Raw sensor" estimator — feed the autopilot
                                 the unfiltered LANDING_TARGET position.
                                 Simpler than the Kalman estimator (=1) and
                                 sufficient at our update rates.
          • PLND_YAW_ALIGN = 0   No yaw rotation between camera and vehicle
                                 frames (centidegrees).  Our downward camera
                                 is mounted aligned with the airframe X-axis.

        Wrapped in try/except per param so a single failure does not abort
        the whole startup sequence.
        """
        print("[INFO] Enabling ArduCopter PrecLand parameters...")
        params = {
            "PLND_ENABLED":   1,
            "PLND_TYPE":      1,
            "PLND_EST_TYPE":  0,
            "PLND_YAW_ALIGN": 0,
        }
        for name, value in params.items():
            try:
                self.master.mav.param_set_send(
                    self.master.target_system, self.master.target_component,
                    name.encode(), float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                )
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
                print(f"  {name} = {value}  ({msg.to_dict() if msg else 'no ack'})")
            except Exception as e:
                print(f"  Failed to set {name}: {e}")
        self.master.wait_heartbeat()

    # ── Takeoff ──────────────────────────────────────────────────────────────

    def takeoff_to_altitude(self, meters, pump_fn=None):
        """Send NAV_TAKEOFF and block until within 0.3 m of target.

        Altitude reasoning
        ──────────────────
        ``LOCAL_POSITION_NED.z`` is reported relative to the EKF/local origin,
        NOT the physical takeoff point.  On airframes with a stale GPS/EKF the
        origin can be hundreds of metres off; in one real-flight log we saw
        the FIRST LOCAL_POSITION_NED after MAV_CMD_NAV_TAKEOFF report
        -z = 339.88 m while the drone was still on the ground, which made the
        old "altitude >= meters - 0.3" check pass instantly and the script
        proceeded to PATROL with a grounded, armed drone.

        We therefore seed ``self.takeoff_z_origin`` from the freshest
        LOCAL_POSITION_NED *before* sending NAV_TAKEOFF and use
        ``relative_alt = -(msg.z - z0)`` as the climb metric everywhere
        downstream.  We also read COMMAND_ACK so silent FCU rejections
        (common without solid GPS lock) abort the mission instead of leaving
        the drone armed-on-ground.

        ``pump_fn`` is an optional zero-arg callable invoked between altitude
        polls so the camera preview keeps refreshing during the climb.
        """
        # ── Step 1: seed the z anchor from the freshest LOCAL_POSITION_NED ──
        # We need at least one fresh sample BEFORE issuing NAV_TAKEOFF;
        # without it we have no baseline to subtract and altitude readings
        # downstream are meaningless.
        z0 = None
        for _ in range(6):
            msg = self.master.recv_match(type="LOCAL_POSITION_NED",
                                         blocking=True, timeout=0.5)
            if msg is not None:
                self.last_pos["x"]  = msg.x
                self.last_pos["y"]  = msg.y
                self.last_pos["z"]  = msg.z
                self.last_pos["vx"] = msg.vx
                self.last_pos["vy"] = msg.vy
                self.last_pos["vz"] = msg.vz
                self.last_pos["t"]  = time.time()
                z0 = msg.z
        if z0 is None:
            raise RuntimeError(
                "No LOCAL_POSITION_NED received within 3 s — cannot anchor "
                "takeoff altitude.  Check the FCU telemetry stream."
            )
        self.takeoff_z_origin = z0
        print(f"[INFO] Takeoff anchor captured: raw -z = {-z0:.2f} m "
              f"(EKF/local origin offset; relative altitude will be reported "
              f"against this baseline)")

        # ── Step 2: advisory EKF/health check (does NOT abort) ──────────────
        # Drain whatever SYS_STATUS / EKF_STATUS_REPORT happens to be in the
        # buffer right now and surface obvious red flags so the operator can
        # correlate a later NAV_TAKEOFF rejection with degraded EKF health.
        # Wrapped in try/except so older pymavlink dialects without these
        # attributes don't break the main flow.
        try:
            ekf = self.master.recv_match(type="EKF_STATUS_REPORT",
                                         blocking=False)
            if ekf is not None:
                flags = getattr(ekf, "flags", None)
                vel_var = getattr(ekf, "velocity_variance", 0.0)
                # Bit 8 = EKF_PRED_POS_HORIZ_ABS, bit 9 = EKF_POS_HORIZ_ABS.
                if flags is not None and (
                    not (flags & (1 << 8)) or not (flags & (1 << 9))
                ):
                    print("[WARN] EKF reports degraded position estimate — "
                          "takeoff may be rejected by FCU "
                          f"(flags=0x{flags:04x})")
                if vel_var is not None and vel_var > 1.0:
                    print("[WARN] EKF reports degraded position estimate — "
                          "takeoff may be rejected by FCU "
                          f"(velocity_variance={vel_var:.2f})")
            # Also drain a SYS_STATUS if available — purely informational.
            self.master.recv_match(type="SYS_STATUS", blocking=False)
        except Exception as e:
            # Any error here is advisory-only; never block takeoff on it.
            print(f"[WARN] EKF health probe skipped: {e}")

        # ── Step 3: issue NAV_TAKEOFF ────────────────────────────────────────
        print(f"[INFO] Taking off to {meters} m...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, meters,
        )

        # ── Step 4: read COMMAND_ACK for NAV_TAKEOFF (cmd 22) ───────────────
        # Without this, a silent FCU rejection (very common when GPS/EKF is
        # not healthy, or pre-arm checks fail post-arm) goes undetected and
        # we end up "monitoring altitude" of a stationary drone.
        ack_seen   = False
        ack_result = None
        ack_deadline = time.time() + 3.0
        while time.time() < ack_deadline:
            ack = self.master.recv_match(type="COMMAND_ACK",
                                         blocking=True, timeout=0.5)
            if ack is None:
                continue
            if getattr(ack, "command", None) == \
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                ack_seen   = True
                ack_result = ack.result
                break
        if ack_seen:
            if ack_result != 0:
                print(f"[ERROR] Takeoff rejected by FCU "
                      f"(COMMAND_ACK result={ack_result}) — most likely "
                      f"EKF/GPS not healthy or pre-arm checks failing")
                raise RuntimeError(
                    f"NAV_TAKEOFF rejected by FCU (result={ack_result})"
                )
            print("[INFO] NAV_TAKEOFF accepted by FCU")
        else:
            print("[WARN] No COMMAND_ACK for NAV_TAKEOFF within 3 s — "
                  "proceeding but watching for liftoff failure")

        # ── Step 5: monitor climb against the anchored baseline ─────────────
        start = time.time()
        liftoff_deadline = start + 6.0   # by 6 s we expect SOME vertical motion
        liftoff_seen = False
        last_relative = 0.0
        while time.time() - start < 30:
            msg = self.master.recv_match(type="LOCAL_POSITION_NED",
                                         blocking=True, timeout=1)
            if msg is not None:
                self.last_pos["x"]  = msg.x
                self.last_pos["y"]  = msg.y
                self.last_pos["z"]  = msg.z
                self.last_pos["vx"] = msg.vx
                self.last_pos["vy"] = msg.vy
                self.last_pos["vz"] = msg.vz
                self.last_pos["t"]  = time.time()

                # Relative altitude = climb above the takeoff anchor.  Raw -z
                # is printed alongside so the EKF-origin offset is visible
                # in the log on every flight.
                relative_alt = -(msg.z - z0)
                last_relative = relative_alt
                print(f"[INFO] Altitude: {relative_alt:+.2f} m relative  "
                      f"(raw -z = {-msg.z:.2f} m, anchor = {-z0:.2f} m)")

                if abs(relative_alt) > 0.3:
                    liftoff_seen = True

                if relative_alt >= meters - 0.3:
                    print(f"[INFO] Target altitude reached "
                          f"({relative_alt:+.2f} m relative)")
                    return

            # Drone-never-moved guard.  If 6 s after takeoff the drone has
            # not visibly climbed, the FCU almost certainly dropped the
            # takeoff (no ACK case above) — abort now rather than continue
            # to PATROL with a grounded, armed drone.
            if not liftoff_seen and time.time() > liftoff_deadline:
                print("[ERROR] Drone never lifted off — relative altitude "
                      f"{last_relative:+.2f} m after 6 s.  The FCU most "
                      "likely silently rejected NAV_TAKEOFF (EKF/GPS not "
                      "healthy or pre-arm checks failing).")
                raise RuntimeError(
                    "Drone never lifted off after NAV_TAKEOFF (no vertical "
                    "motion within 6 s)"
                )

            if pump_fn is not None:
                pump_fn()
            time.sleep(0.1)
        print("[WARN] Altitude timeout — proceeding anyway")

    # ── Velocity control (body frame) ────────────────────────────────────────

    def send_velocity(self, vx, vy, vz):
        """Send a body-frame velocity command (mirrors patrol_landing.py)."""
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            TYPEMASK_VELOCITY_ONLY,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )
        self.last_cmd["vx"] = vx
        self.last_cmd["vy"] = vy
        self.last_cmd["vz"] = vz

    # ── TRACK phase: velocity-PD lock over the tag ───────────────────────────

    def track_velocity_command(self, body_x, body_y):
        """Drive the drone to (0, 0) in body-frame XY using a light EMA + PD.

        Used by the TRACK phase between PATROL and PRECISION_LAND so the
        drone is centered above the tag BEFORE any descent begins.  The
        autopilot's own PrecLand controller then takes over.

        Vertical velocity is held at TRACK_VZ_HOLD (0 m/s by default) —
        descent is the next phase's job, not this one's.

        Filter / derivative state mirrors
        stationary_landing_controller.adjust_velocity_and_send():
          * Re-seed prev state if it is None or older than 0.5 s.  A
            stale buffer would produce a huge spurious D-term spike on
            the next valid frame.
          * Apply alpha=0.5 EMA on the body-frame inputs.
          * Compute the derivative on the FILTERED values; the P-term
            uses the raw input as in the spec.

        Returns the (vx, vy, vz) actually sent so the caller can log it.
        """
        now = time.time()

        if (self._track_prev_x is None
                or self._track_prev_t is None
                or (now - self._track_prev_t) > 0.5):
            self._track_prev_x = body_x
            self._track_prev_y = body_y
            self._track_prev_t = now

        alpha = 0.5
        filt_x = alpha * self._track_prev_x + (1 - alpha) * body_x
        filt_y = alpha * self._track_prev_y + (1 - alpha) * body_y

        # Clamp dt to suppress div-by-zero on the seeded frame and
        # D-term spikes on frame skips.
        dt = max(min(now - self._track_prev_t, 0.2), 0.01)
        dx = (filt_x - self._track_prev_x) / dt
        dy = (filt_y - self._track_prev_y) / dt

        vx = TRACK_Kp_XY * body_x + TRACK_Kd_XY * dx
        vy = TRACK_Kp_XY * body_y + TRACK_Kd_XY * dy

        vx = max(min(vx, TRACK_MAX_V_XY), -TRACK_MAX_V_XY)
        vy = max(min(vy, TRACK_MAX_V_XY), -TRACK_MAX_V_XY)
        vz = TRACK_VZ_HOLD

        self._track_prev_x = filt_x
        self._track_prev_y = filt_y
        self._track_prev_t = now

        self.send_velocity(vx, vy, vz)
        return vx, vy, vz

    # ── Position control (local NED) ─────────────────────────────────────────

    def goto_ned(self, x, y, z, tolerance=GOTO_TOLERANCE,
                 timeout=GOTO_TIMEOUT, pump_fn=None):
        """Fly to absolute NED (x, y, z), blocking until reached or timed out.

        Uses SET_POSITION_TARGET_LOCAL_NED with the position-only typemask
        (TYPEMASK_POSITION_ONLY).  The autopilot handles the trajectory
        internally; we just resend the target every 0.5 s and watch our
        LOCAL_POSITION_NED until the horizontal+vertical error is within
        ``tolerance``.

        IMPORTANT — z is ABSOLUTE NED (the same frame as
        ``LOCAL_POSITION_NED.z``), not a relative-to-takeoff altitude.
        On airframes with a stale EKF/local origin the takeoff point is
        offset from z=0 by hundreds of metres (real flight log: ~340 m),
        so callers must add the offset themselves:

            target_z = controller.takeoff_z_origin - desired_relative_alt

        Passing ``-desired_relative_alt`` directly will command a goto to
        absolute z=−desired_relative_alt, which on a stale-origin airframe
        is hundreds of metres above (or below!) the actual takeoff point.
        """
        print(f"[INFO] goto_ned → ({x:+.2f}, {y:+.2f}, {z:+.2f})  "
              f"tol={tolerance:.2f} m")
        start = time.time()
        last_send = 0.0
        while time.time() - start < timeout:
            now = time.time()
            if now - last_send > 0.5:
                self.master.mav.set_position_target_local_ned_send(
                    0,
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    TYPEMASK_POSITION_ONLY,
                    x, y, z,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0,
                )
                last_send = now

            cur_x, cur_y, cur_z, _, _, _ = self.get_local_position()
            if cur_x is not None:
                dx, dy, dz = cur_x - x, cur_y - y, cur_z - z
                err = math.sqrt(dx * dx + dy * dy + dz * dz)
                if err < tolerance:
                    print(f"[INFO] goto_ned reached (err={err:.2f} m)")
                    return True

            if pump_fn is not None:
                pump_fn()
            time.sleep(0.1)
        print(f"[WARN] goto_ned timed out after {timeout:.1f} s — continuing")
        return False

    # ── Stabilization (post-takeoff hold verification) ───────────────────────

    def wait_stabilized(self, target_alt, pump_fn=None):
        """Verify altitude + lateral drift + horizontal velocity are stable.

        Returns the (x_home, y_home) NED position captured at the end of
        stabilization — this is the "original point" the user wants used as
        the anchor for the patrol and the post-loss relocate.
        """
        print("[INFO] Stabilizing — waiting for hold to settle...")
        x_origin, y_origin = None, None
        ok_since = None
        start = time.time()
        last_print = 0.0
        warned_no_anchor = False

        while time.time() - start < STABILIZE_TIMEOUT_SECONDS:
            cur_x, cur_y, cur_z, cur_vx, cur_vy, _ = self.get_local_position()
            if cur_x is None:
                # No telemetry yet — keep pumping camera & wait.
                self.send_velocity(0, 0, 0)
                if pump_fn is not None:
                    pump_fn()
                time.sleep(0.1)
                continue

            # Origin = first position we see post-takeoff.  Lateral drift is
            # measured relative to it.
            if x_origin is None:
                x_origin, y_origin = cur_x, cur_y

            # Use the takeoff anchor so altitude matches takeoff_to_altitude's
            # frame.  Falling back to absolute -cur_z (the old behaviour) is
            # only safe when the EKF origin happens to coincide with the
            # ground; surface a one-time WARN so the regression is visible.
            if self.takeoff_z_origin is not None:
                relative_alt = -(cur_z - self.takeoff_z_origin)
            else:
                if not warned_no_anchor:
                    print("[WARN] No takeoff anchor — stabilize altitude "
                          "check uses absolute -z")
                    warned_no_anchor = True
                relative_alt = -cur_z
            alt_err  = abs(relative_alt - target_alt)
            drift    = math.sqrt((cur_x - x_origin) ** 2 +
                                 (cur_y - y_origin) ** 2)
            hspd     = math.sqrt(cur_vx ** 2 + cur_vy ** 2)

            alt_ok    = alt_err  < STABILIZE_ALT_TOLERANCE
            drift_ok  = drift    < STABILIZE_DRIFT_TOLERANCE
            vel_ok    = hspd     < STABILIZE_VEL_TOLERANCE
            all_ok    = alt_ok and drift_ok and vel_ok

            # Active hold — body-frame zero command pins the position while
            # we wait.  Doing nothing leaves AP coasting on whatever the
            # NAV_TAKEOFF command last produced and can let the drone drift.
            self.send_velocity(0, 0, 0)

            now = time.time()
            if now - last_print > 0.5:
                print(f"[INFO] STABILIZE alt={relative_alt:+.2f} m "
                      f"(raw -z={-cur_z:.2f})  alt_err={alt_err:+.2f} m  "
                      f"drift={drift:.2f} m  hspd={hspd:.2f} m/s  "
                      f"{'OK' if all_ok else 'WAIT'}")
                last_print = now

            if all_ok:
                if ok_since is None:
                    ok_since = now
                elif now - ok_since >= STABILIZE_HOLD_SECONDS:
                    print(f"[INFO] Stabilized — origin=({cur_x:+.2f}, "
                          f"{cur_y:+.2f})  alt={relative_alt:+.2f} m")
                    return cur_x, cur_y
            else:
                ok_since = None

            if pump_fn is not None:
                pump_fn()
            time.sleep(0.05)

        print("[WARN] Stabilization timed out — proceeding with current pos")
        cur_x, cur_y, _, _, _, _ = self.get_local_position()
        if cur_x is None:
            return 0.0, 0.0
        return cur_x, cur_y

    # ── Precision-landing: LANDING_TARGET publication ────────────────────────

    def send_landing_target(self, body_x, body_y, body_z):
        """Publish a LANDING_TARGET to the FCU in body-frame.

        The ArduCopter PrecLand companion driver only honors two payload
        shapes from a MAVLink-source companion: the angular form
        (angle_x, angle_y, distance) computed from the camera optical
        axis, OR — on AC ≥ 4.1 — the position form when ``frame`` is
        ``MAV_FRAME_BODY_FRD`` and ``position_valid`` is 1.  An earlier
        version of this method sent ``MAV_FRAME_LOCAL_NED`` with absolute
        NED coordinates and ``angle_x = angle_y = 0``; the autopilot
        silently ignored the position payload and fell back to "target
        straight below" (the zero angles), so the drone descended
        vertically without any lateral correction.  Real-flight log:
        the tag drifted out of FOV after ~3 m of descent.

        Inputs are body-frame metres relative to the drone:
            body_x  forward  (+ = nose direction)
            body_y  right    (+ = starboard)
            body_z  down     (+ = below the drone — downward camera)

        We compute:
            distance = ||(body_x, body_y, body_z)||
            angle_x  = atan2(body_x, body_z)   forward offset, radians
            angle_y  = atan2(body_y, body_z)   right offset, radians
        ``body_z`` is clamped to a small positive epsilon before atan2 so
        the angles stay defined during the brief moment before the
        autopilot commits to descent (when the tag may briefly read at or
        slightly above the camera plane due to pose-estimation noise).

        Rate-limited to LANDING_TARGET_RATE_HZ (≈10 Hz) — calling more
        frequently just floods the serial link.

        Two pymavlink signatures exist in the wild:
            * MAVLink-2 (14 args, includes x/y/z + q + type + position_valid)
            * Older MAVLink-1 (9 args, x/y/z/q/type/position_valid absent)
        We try the modern form first and fall back to the legacy form,
        keeping the angular form populated in BOTH paths so older builds
        still get a usable target instead of "straight below".
        """
        now = time.time()
        if now - self._last_lt_send < LANDING_TARGET_MIN_DT:
            return False
        self._last_lt_send = now

        distance = math.sqrt(body_x ** 2 + body_y ** 2 + body_z ** 2)
        # body_z > 0 means tag is below the drone (downward-facing camera).
        # Clamp to a small positive epsilon to avoid undefined angles when
        # the tag is at or above the camera plane.
        safe_bz = max(body_z, 1e-3)
        angle_x = math.atan2(body_x, safe_bz)
        angle_y = math.atan2(body_y, safe_bz)
        time_usec = int(now * 1e6)

        try:
            self.master.mav.landing_target_send(
                time_usec,
                0,                                              # target_num
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                angle_x, angle_y,
                distance,
                TAG_SIZE, TAG_SIZE,                             # size_x, y
                body_x, body_y, body_z,
                (1.0, 0.0, 0.0, 0.0),                           # identity quat
                mavutil.mavlink.LANDING_TARGET_TYPE_VISION_OTHER,
                1,                                              # position_valid
            )
        except TypeError:
            # Older pymavlink with the 9-arg signature — fall back to the
            # angular form on the same body-frame frame.  The angular
            # payload IS still correct on this path (unlike the previous
            # implementation which zeroed it).
            try:
                self.master.mav.landing_target_send(
                    time_usec,
                    0,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    angle_x, angle_y,
                    distance,
                    TAG_SIZE, TAG_SIZE,
                )
                print("[WARN] Falling back to 9-arg LANDING_TARGET — "
                      "pymavlink is older than MAVLink-2; positional "
                      "pose dropped (angular form retained)")
            except Exception as e:
                print(f"[WARN] LANDING_TARGET send failed: {e}")
                return False
        except Exception as e:
            print(f"[WARN] LANDING_TARGET send failed: {e}")
            return False

        self.last_cmd["lt_x"] = body_x
        self.last_cmd["lt_y"] = body_y
        self.last_cmd["lt_z"] = body_z
        return True

    # ── Frame conversions ────────────────────────────────────────────────────

    @staticmethod
    def camera_to_body(cam_x, cam_y, cam_z):
        """Downward-facing camera → body frame (matches existing code)."""
        return -cam_y, cam_x, cam_z

    def tag_camera_to_ned(self, cam_x, cam_y, cam_z):
        """Convert a tag pose in the camera frame into the autopilot's
        LOCAL_NED frame.

        Steps:
          1. camera → body via camera_to_body() (downward camera convention).
          2. body → NED by adding the drone's current LOCAL_POSITION_NED.

        NB: step 2 implicitly assumes the body and NED frames are aligned
        in yaw (i.e. the drone is heading North-ish at the moment of
        capture).  For yaws that differ significantly, a proper rotation
        by ATTITUDE.yaw would be required; the user explicitly specified
        this simplified transform, so it's flagged here for visibility.
        """
        body_x, body_y, body_z = self.camera_to_body(cam_x, cam_y, cam_z)
        drone_x, drone_y, drone_z, _, _, _ = self.get_local_position()
        if drone_x is None:
            # No NED yet — best-effort: return body coords as-is so something
            # still reaches the autopilot.
            return body_x, body_y, body_z
        return drone_x + body_x, drone_y + body_y, drone_z + body_z


# ─────────────────────────────────────────────────────────────────────────────
# Visualization
# ─────────────────────────────────────────────────────────────────────────────

def draw_tag(frame, tag):
    """Outline the tag (green) and mark its center (red dot)."""
    corners = tag.corners.astype(int)
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                 (0, 255, 0), 2)
    cv2.circle(frame, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)


def draw_overlay(frame, state):
    """Render the full HUD onto ``frame`` based on the shared state dict.

    Layout: text in the top-left corner, one line per field, font scale
    0.45 (matches patrol_landing.py).  Colours: yellow for phase/mode,
    cyan for telemetry, green for "tag visible", red for "tag lost",
    orange for the active motor signal.
    """
    phase     = state.get("phase", "?")
    flightmode = state.get("flightmode", "?")
    armed     = state.get("armed", False)
    altitude  = state.get("altitude")
    leg_label = state.get("leg_label", "")
    tag_seen  = state.get("tag_visible", False)
    time_lost = state.get("time_lost", 0.0)
    cmd       = state.get("cmd", {})

    color_phase   = (  0, 255, 255)
    color_motors  = (  0, 255,   0) if armed else (0, 0, 255)
    color_tag     = (  0, 255,   0) if tag_seen else (0, 0, 255)
    color_cmd     = (  0, 200, 255)
    color_white   = (255, 255, 255)

    alt_text = f"{altitude:+.2f} m" if altitude is not None else "  n/a"

    lines = [
        (f"PHASE   {phase}",                        color_phase),
        (f"MODE    {flightmode}",                   color_phase),
        (f"MOTORS  {'ARMED' if armed else 'DISARMED'}", color_motors),
        (f"ALT     {alt_text}",                     color_white),
        (f"LEG     {leg_label}",                    color_white),
    ]

    # In PRECISION_LAND / SEARCH the motor-driving signal is the
    # LANDING_TARGET we publish, not a velocity command.  Show whichever
    # is active so the operator can see what the autopilot is reacting to.
    # (Caveat: we don't have per-motor PWM telemetry; the published command
    # is the closest stand-in for "which motors are being used".)
    # The LT payload is body-frame (the autopilot's BODY_FRD), so label
    # it explicitly — earlier versions stored absolute NED here, and the
    # mismatch made HUD-debugging the descent confusing.
    if phase in ("PRECISION_LAND", "SEARCH") and cmd.get("lt_x") is not None:
        lines.append((
            f"LT body x={cmd['lt_x']:+.2f} y={cmd['lt_y']:+.2f} "
            f"z={cmd['lt_z']:+.2f}",
            color_cmd,
        ))
    else:
        lines.append((
            f"CMD     vx={cmd.get('vx', 0.0):+.2f} "
            f"vy={cmd.get('vy', 0.0):+.2f} "
            f"vz={cmd.get('vz', 0.0):+.2f}",
            color_cmd,
        ))

    if tag_seen:
        lines.append(("TAG     VISIBLE", color_tag))
    else:
        lines.append((f"TAG     LOST ({time_lost:.1f}s)", color_tag))

    for idx, (text, color) in enumerate(lines):
        cv2.putText(frame, text, (10, 20 + idx * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


# ─────────────────────────────────────────────────────────────────────────────
# Camera-pump helper — keeps the live preview window updating during EVERY
# phase (pre-arm, arming, takeoff, stabilization, patrol, precision-landing,
# search, touchdown).  The window must be visible "as soon as the program
# runs" per the user spec, so this is called from the moment the pipeline is
# up — long before any flight command is issued.
# ─────────────────────────────────────────────────────────────────────────────

def make_pump(q_rgb, detector, controller, state):
    """Build a closure that pulls the latest frame, detects (if requested),
    refreshes the overlay HUD, and pumps cv2.waitKey so the window stays live.

    Returns a zero-arg function suitable for handing to long-blocking
    controller methods (takeoff_to_altitude, wait_stabilized, goto_ned)
    so the camera does not freeze during those phases.

    ``state`` is mutated in-place — it is the single shared dictionary
    everything writes to and the overlay reads from.
    """
    def pump(detect=False):
        # 1. Drain MAVLink for the freshest telemetry first.  Without this
        #    the overlay shows stale altitude/mode/armed values.
        controller._drain_messages()
        state["flightmode"] = controller.master.flightmode
        state["armed"]      = controller.master.motors_armed()
        if controller.last_pos["z"] is not None:
            if controller.takeoff_z_origin is not None:
                state["altitude"] = -(controller.last_pos["z"]
                                      - controller.takeoff_z_origin)
            else:
                state["altitude"] = -controller.last_pos["z"]
        state["cmd"] = controller.last_cmd

        # 2. Pull the most recent camera frame (if any).  tryGet() never
        #    blocks; if the queue is empty we render the last cached frame
        #    so the overlay still updates.
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            state["frame"] = in_rgb.getCvFrame()

        if state.get("frame") is None:
            # No frame yet — nothing to draw.  Pump cv2 events anyway to
            # keep the window manager from declaring us unresponsive.
            cv2.waitKey(1)
            return None

        # 3. Optional detection.  We re-detect only when the caller asks
        #    (the heavy preprocessing pipeline is too expensive to run on
        #    every camera-pump call during e.g. arming).
        tag = None
        if detect and detector is not None:
            tag = detector.get_tag_detection(state["frame"])
            if tag is not None:
                state["tag_visible"]   = True
                state["last_tag_time"] = time.time()
                state["last_tag"]      = tag
            else:
                state["tag_visible"] = False

        # 4. Compose the displayed frame: tag overlay (if we just saw one)
        #    plus the HUD text.  Always copy so the queue's frame isn't
        #    annotated in place (some DepthAI versions share the buffer).
        display = state["frame"].copy()
        if tag is not None:
            draw_tag(display, tag)

        # Update time-since-lost so the overlay can show it during gaps.
        last_t = state.get("last_tag_time", 0.0)
        if state.get("tag_visible"):
            state["time_lost"] = 0.0
        else:
            state["time_lost"] = (time.time() - last_t) if last_t else 0.0

        draw_overlay(display, state)
        cv2.imshow(WINDOW_TITLE, display)
        cv2.waitKey(1)
        return tag

    return pump


# ─────────────────────────────────────────────────────────────────────────────
# Patrol — body-frame velocity legs, with mid-patrol tag-detection abort.
# Returns (last_known_x, last_known_y) if the tag was found during patrol,
# or None if the full box was flown without seeing the tag.
# ─────────────────────────────────────────────────────────────────────────────

def run_box_patrol(controller, pump, state, leg_offset=0):
    """Fly the 5-ft-per-leg box patrol.  ``leg_offset`` lets the re-search
    invocation label legs as 5/8, 6/8, etc. if you want — currently unused
    but kept for clarity.
    """
    print("[INFO] Phase: PATROL")
    state["phase"] = "PATROL"
    n_segments = len(PATROL_SEGMENTS)

    for i, (vx, vy, vz, duration, label) in enumerate(PATROL_SEGMENTS):
        leg_idx = leg_offset + i + 1
        print(f"[INFO] Patrol leg {leg_idx}/{leg_offset + n_segments}: "
              f"{label} for {duration:.1f}s")
        t_start  = time.time()
        last_log = 0.0

        while time.time() - t_start < duration:
            # FCU disarmed mid-patrol = bail out — RC failsafe / ground hit.
            controller._drain_messages()
            if not controller.master.motors_armed():
                print("[INFO] Motors disarmed mid-patrol — exiting.")
                return None

            controller.send_velocity(vx, vy, vz)

            remaining = duration - (time.time() - t_start)
            state["leg_label"] = f"LEG {leg_idx}/{leg_offset + n_segments} {label.upper()}  ({remaining:.1f}s)"

            tag = pump(detect=True)
            if tag is not None:
                # Abort the patrol the instant we see the tag — every extra
                # leg flies us further from it.
                pos = controller.get_local_position()
                if pos[0] is not None:
                    last_x, last_y = pos[0], pos[1]
                else:
                    last_x, last_y = 0.0, 0.0
                print(f"[INFO] Tag acquired during patrol — last known "
                      f"NED=({last_x:+.2f}, {last_y:+.2f})")
                controller.send_velocity(0, 0, 0)
                return last_x, last_y

            now = time.time()
            if now - last_log > 1.0:
                print(f"[INFO] PATROL leg {leg_idx} {label}  "
                      f"remaining={remaining:.1f}s")
                last_log = now

            time.sleep(0.05)

    print("[INFO] Patrol complete — hovering")
    controller.send_velocity(0, 0, 0)
    state["leg_label"] = ""
    time.sleep(1.5)
    return None


# ─────────────────────────────────────────────────────────────────────────────
# TRACK phase — runs between PATROL and PRECISION_LAND.  Holds station above
# the tag with velocity-PD control for TRACK_DURATION_S seconds, regardless
# of whether the drone has perfectly centered itself.  This guarantees the
# autopilot's PrecLand controller starts from a near-centered hover; in the
# old flow the very first LANDING_TARGET went out from a marginal angle
# and descent began before the lateral loop converged.
#
# Returns:
#   "READY"     — duration elapsed, hand off to PRECISION_LAND
#   "TAG_LOST"  — tag missing > TRACK_LOSS_TIMEOUT_S; caller should SEARCH
#   "TOUCHDOWN" — motors disarmed mid-track (defensive — at altitude this
#                 should not happen, but treat it as a clean exit if it does)
# ─────────────────────────────────────────────────────────────────────────────

def track_tag(controller, pump, state):
    print("[INFO] Phase: TRACK")
    state["phase"]     = "TRACK"
    state["leg_label"] = f"TRACK 0.0/{TRACK_DURATION_S:.0f}s"

    start         = time.time()
    last_tag_time = time.time()
    last_log      = 0.0

    while True:
        controller._drain_messages()
        if not controller.master.motors_armed():
            print("[INFO] Motors disarmed during TRACK — treating as touchdown.")
            return "TOUCHDOWN"

        elapsed = time.time() - start
        if elapsed >= TRACK_DURATION_S:
            controller.send_velocity(0.0, 0.0, 0.0)
            print(f"[INFO] TRACK duration met ({elapsed:.1f}s) — handing "
                  "off to PRECISION_LAND")
            return "READY"

        tag = pump(detect=True)

        if tag is not None:
            last_tag_time = time.time()
            t = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])
            body_x, body_y, body_z = PrecisionLandingController.camera_to_body(
                cam_x, cam_y, cam_z,
            )

            vx, vy, vz = controller.track_velocity_command(body_x, body_y)

            state["leg_label"] = (
                f"TRACK {elapsed:.1f}/{TRACK_DURATION_S:.0f}s "
                f"err=({body_x:+.2f},{body_y:+.2f})"
            )

            now = time.time()
            if now - last_log > 1.0:
                print(f"[INFO] TRACK t={elapsed:.1f}/{TRACK_DURATION_S:.0f}s  "
                      f"body=({body_x:+.2f},{body_y:+.2f},{body_z:+.2f})  "
                      f"v=({vx:+.2f},{vy:+.2f},{vz:+.2f})")
                last_log = now
        else:
            time_lost = time.time() - last_tag_time
            if time_lost > TRACK_LOSS_TIMEOUT_S:
                print(f"[WARN] Tag lost for {time_lost:.1f}s during TRACK — "
                      "bailing to SEARCH")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "TAG_LOST"
            controller.send_velocity(0.0, 0.0, 0.0)
            state["leg_label"] = (
                f"TRACK {elapsed:.1f}/{TRACK_DURATION_S:.0f}s "
                f"LOST {time_lost:.1f}s"
            )

        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Precision-landing phase.  Returns one of:
#   "TOUCHDOWN"  — motors auto-disarmed; mission complete
#   "TAG_LOST"   — tag missing > TAG_LOSS_TIMEOUT; caller should re-search
# ─────────────────────────────────────────────────────────────────────────────

def precision_land(controller, pump, state):
    print("[INFO] Phase: PRECISION_LAND")
    state["phase"] = "PRECISION_LAND"
    last_tag_time = time.time()
    mode_switched = False
    sent_initial_lt = False

    while True:
        # Drain telemetry & exit cleanly on disarm.  ArduCopter auto-disarms
        # on touchdown (LAND mode ground-detection); seeing motors_armed
        # drop to False is the most reliable touchdown indicator.
        controller._drain_messages()
        if not controller.master.motors_armed():
            print("[INFO] Motors disarmed — touchdown detected.")
            return "TOUCHDOWN"

        tag = pump(detect=True)

        if tag is not None:
            last_tag_time = time.time()
            t = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])

            # Body-frame is what ArduCopter PrecLand actually consumes
            # (MAV_FRAME_BODY_FRD).  No NED conversion in this path.
            body_x, body_y, body_z = PrecisionLandingController.camera_to_body(
                cam_x, cam_y, cam_z,
            )

            sent = controller.send_landing_target(body_x, body_y, body_z)
            if sent:
                safe_bz = max(body_z, 1e-3)
                ang_x = math.atan2(body_x, safe_bz)
                ang_y = math.atan2(body_y, safe_bz)
                dist = math.sqrt(body_x ** 2 + body_y ** 2 + body_z ** 2)
                if (controller.last_pos["z"] is not None and
                        controller.takeoff_z_origin is not None):
                    rel_alt = -(controller.last_pos["z"]
                                - controller.takeoff_z_origin)
                    raw_neg_z = -controller.last_pos["z"]
                    print(f"[INFO] LANDING_TARGET body=({body_x:+.2f}, "
                          f"{body_y:+.2f}, {body_z:+.2f}) "
                          f"ang=({ang_x:+.2f},{ang_y:+.2f}) rad  "
                          f"dist={dist:.2f} m  alt={rel_alt:+.2f} m relative "
                          f"(raw -z={raw_neg_z:.2f})")
                else:
                    raw_neg_z = (-controller.last_pos["z"]
                                 if controller.last_pos["z"] is not None
                                 else float("nan"))
                    print(f"[INFO] LANDING_TARGET body=({body_x:+.2f}, "
                          f"{body_y:+.2f}, {body_z:+.2f}) "
                          f"ang=({ang_x:+.2f},{ang_y:+.2f}) rad  "
                          f"dist={dist:.2f} m  raw -z={raw_neg_z:.2f}")
                sent_initial_lt = True

            # Hand off to autopilot ONLY after the first LANDING_TARGET has
            # actually gone out.  If we switch to LAND mode first, the very
            # first descent step happens without a target and the autopilot
            # falls back to a plain straight-down LAND.
            if sent_initial_lt and not mode_switched:
                controller.change_flight_mode("LAND")
                mode_switched = True

        else:
            elapsed = time.time() - last_tag_time
            if elapsed > TAG_LOSS_TIMEOUT:
                # Compute relative altitude (against the takeoff anchor) so
                # we can decide whether the loss is "drone too high; tag
                # actually gone" vs "drone almost on top of the tag and the
                # tag has simply left the FOV".  In the latter case, the
                # autopilot is already in LAND mode and ground-detection
                # will auto-disarm on touchdown — committing to LAND is
                # safer than re-flying the search box at <2 m AGL.
                relative_alt = None
                if (controller.last_pos["z"] is not None and
                        controller.takeoff_z_origin is not None):
                    relative_alt = -(controller.last_pos["z"]
                                     - controller.takeoff_z_origin)
                if (relative_alt is not None and
                        relative_alt < TAG_TOO_CLOSE_ALT_M):
                    print(f"[INFO] Tag lost at low altitude "
                          f"({relative_alt:.2f} m) — committing to LAND "
                          "descent until touchdown")
                    # Stay in LAND mode (we're already in it).  Loop until
                    # motors disarm or a generous timeout (30 s) so a
                    # stuck mode does not leave us hanging forever.
                    land_start = time.time()
                    while time.time() - land_start < 30:
                        controller._drain_messages()
                        if not controller.master.motors_armed():
                            print("[INFO] Motors disarmed — touchdown.")
                            return "TOUCHDOWN"
                        pump()
                        time.sleep(0.1)
                    return "TOUCHDOWN"
                print(f"[WARN] Tag lost for {elapsed:.1f}s during "
                      "PRECISION_LAND")
                return "TAG_LOST"

        # ~50 Hz tick — well above the 10 Hz LANDING_TARGET send rate but
        # comfortable on a Pi while still leaving headroom for detection.
        time.sleep(0.02)


# ─────────────────────────────────────────────────────────────────────────────
# Search / re-locate — climb back, fly to last known tag spot, re-patrol.
# ─────────────────────────────────────────────────────────────────────────────

def search_and_relocate(controller, pump, state, last_known_xy):
    """Switch to GUIDED, climb back to TAKEOFF_ALTITUDE, fly to the last
    known marker (x, y), and re-run the box patrol centered there.

    Returns the new (last_known_x, last_known_y) if the tag is re-acquired
    during the re-patrol, or ``None`` if the box completed without a hit.
    """
    print("[INFO] Phase: SEARCH")
    state["phase"]     = "SEARCH"
    state["leg_label"] = "CLIMB"

    controller.change_flight_mode("GUIDED")

    last_x, last_y = last_known_xy

    # Climb back via goto_ned — single call covers both the altitude
    # recovery and the lateral repositioning above the last known marker.
    # goto_ned takes ABSOLUTE NED z, so we must anchor against the EKF
    # origin captured at takeoff.  The previous version passed
    # ``-TAKEOFF_ALTITUDE`` directly, which on a stale-origin airframe
    # commanded a several-hundred-metre descent (real-flight log:
    # actual NED z ≈ -345, target z = -6, ~339 m delta) and would have
    # plowed the drone into the ground if the user had not Ctrl-C'd.
    if controller.takeoff_z_origin is not None:
        target_z = controller.takeoff_z_origin - TAKEOFF_ALTITUDE
    else:
        # Fall back to relative-only without absolute anchor.  This is
        # only correct when the EKF origin coincides with the takeoff
        # point; on stale-origin airframes the goto will be wrong by
        # whatever the offset is.  Surface a [WARN] so the regression
        # is visible in the log.
        print("[WARN] No takeoff anchor — search_and_relocate falling "
              "back to relative-only goto_ned z (may be wrong by the "
              "EKF-origin offset on stale-origin airframes)")
        target_z = -TAKEOFF_ALTITUDE

    controller.goto_ned(
        last_x, last_y, target_z,
        tolerance=GOTO_TOLERANCE, timeout=GOTO_TIMEOUT,
        pump_fn=lambda: pump(detect=True),
    )

    state["leg_label"] = "REPATROL"
    return run_box_patrol(controller, pump, state)


# ─────────────────────────────────────────────────────────────────────────────
# Main — single top-level block (no __main__ guard, matching existing files)
# ─────────────────────────────────────────────────────────────────────────────

with dai.Device() as device:
    print("[INFO] OAK-D started")
    calibration = device.getCalibration()

    detector = AprilTagDetector(calibration)

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
        print("[INFO] Pipeline started — opening preview window...")

        # Build the controller AFTER the camera is live so the OpenCV window
        # appears the moment the program runs (the user explicitly required
        # the window to be visible during pre-arm and arming, not just once
        # the drone is at altitude).
        controller = PrecisionLandingController(CONNECTION_STRING, BAUDRATE)
        controller.request_telemetry_streams()

        # Shared state dict.  Everything writes here, draw_overlay reads it.
        state = {
            "phase":          "INIT",
            "flightmode":     controller.master.flightmode,
            "armed":          False,
            "altitude":       None,
            "leg_label":      "",
            "tag_visible":    False,
            "last_tag_time":  0.0,
            "time_lost":      0.0,
            "frame":          None,
            "last_tag":       None,
            "cmd":            controller.last_cmd,
        }
        pump = make_pump(q_rgb, detector, controller, state)

        # Pump for ~0.5 s so the OpenCV window is on-screen with a phase
        # label BEFORE we touch the FCU.
        for _ in range(10):
            pump()
            time.sleep(0.05)

        # ── Pre-flight: enable PrecLand, set GUIDED, arm, takeoff ──────────
        state["phase"] = "PRECLAND_SETUP"
        controller.enable_precland_params()
        for _ in range(5):
            pump()
            time.sleep(0.05)

        state["phase"] = "GUIDED"
        controller.change_flight_mode("GUIDED")
        for _ in range(5):
            pump()
            time.sleep(0.05)

        state["phase"] = "ARMING"
        controller.arm_motors()
        for _ in range(5):
            pump()
            time.sleep(0.05)

        state["phase"] = "TAKEOFF"
        try:
            controller.takeoff_to_altitude(TAKEOFF_ALTITUDE, pump_fn=pump)
        except RuntimeError as e:
            print(f"[CRITICAL] Takeoff failed: {e}")
            print("[CRITICAL] Aborting mission — switching to LAND for safety")
            controller.change_flight_mode("LAND")
            for _ in range(20):
                pump()
                time.sleep(0.1)
            raise SystemExit(1)

        # ── Stabilization ─────────────────────────────────────────────────
        print("[INFO] Phase: STABILIZE")
        state["phase"] = "STABILIZE"
        x_home, y_home = controller.wait_stabilized(
            TAKEOFF_ALTITUDE, pump_fn=pump,
        )
        print(f"[INFO] Home anchor captured: ({x_home:+.2f}, {y_home:+.2f})")

        # ── First patrol (no last-known yet → patrol around home) ─────────
        last_known = run_box_patrol(controller, pump, state)

        # If patrol completed without a tag, the user's spec doesn't define
        # a recovery; fall through to plain LAND mode at the current spot.
        if last_known is None:
            print("[WARN] Patrol completed without acquiring tag — "
                  "committing to plain LAND at current position")
            state["phase"] = "TOUCHDOWN"
            controller.change_flight_mode("LAND")
            start = time.time()
            while time.time() - start < 30:
                pump()
                if not controller.master.motors_armed():
                    print("[INFO] Motors disarmed — touchdown.")
                    break
                time.sleep(0.1)
        else:
            # ── TRACK → PRECISION_LAND loop, with up to MAX_RESEARCH_ATTEMPTS
            #    re-locate attempts on tag loss in either phase ─────────────
            attempts = 0
            while True:
                # New TRACK phase: hold over the tag for TRACK_DURATION_S
                # before handing control to the autopilot's PrecLand.
                track_result = track_tag(controller, pump, state)
                if track_result == "TOUCHDOWN":
                    state["phase"] = "TOUCHDOWN"
                    break
                if track_result == "TAG_LOST":
                    # Tag lost during TRACK counts as one of our
                    # MAX_RESEARCH_ATTEMPTS re-search attempts.
                    attempts += 1
                    if attempts > MAX_RESEARCH_ATTEMPTS:
                        print(f"[CRITICAL] Exhausted {MAX_RESEARCH_ATTEMPTS}"
                              " re-search attempts (TRACK loss) — falling "
                              "back to plain LAND")
                        state["phase"] = "TOUCHDOWN"
                        controller.change_flight_mode("LAND")
                        fail_start = time.time()
                        while time.time() - fail_start < 30:
                            pump()
                            if not controller.master.motors_armed():
                                print("[INFO] Motors disarmed — touchdown.")
                                break
                            time.sleep(0.1)
                        break
                    print(f"[INFO] Re-search attempt {attempts}/"
                          f"{MAX_RESEARCH_ATTEMPTS} (after TRACK loss)")
                    new_known = search_and_relocate(
                        controller, pump, state, last_known,
                    )
                    if new_known is not None:
                        last_known = new_known
                    continue

                # track_result == "READY" → autopilot PrecLand handoff.
                result = precision_land(controller, pump, state)
                if result == "TOUCHDOWN":
                    state["phase"] = "TOUCHDOWN"
                    break

                # TAG_LOST path — try to re-acquire.
                attempts += 1
                if attempts > MAX_RESEARCH_ATTEMPTS:
                    print(f"[CRITICAL] Exhausted {MAX_RESEARCH_ATTEMPTS} "
                          f"re-search attempts — falling back to plain LAND")
                    state["phase"] = "TOUCHDOWN"
                    controller.change_flight_mode("LAND")
                    fail_start = time.time()
                    while time.time() - fail_start < 30:
                        pump()
                        if not controller.master.motors_armed():
                            print("[INFO] Motors disarmed — touchdown.")
                            break
                        time.sleep(0.1)
                    break

                print(f"[INFO] Re-search attempt {attempts}/{MAX_RESEARCH_ATTEMPTS}")
                new_known = search_and_relocate(controller, pump, state, last_known)
                if new_known is not None:
                    last_known = new_known

        # Final pump so the very last HUD frame is visible briefly before
        # window teardown.
        for _ in range(20):
            pump()
            time.sleep(0.05)

cv2.destroyAllWindows()
