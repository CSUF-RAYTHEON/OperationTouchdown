"""
═══════════════════════════════════════════════════════════════════════════════
 precision_landing_patrol.py
═══════════════════════════════════════════════════════════════════════════════

 What this script does
 ─────────────────────
 Single-file, inline-everything UAV mission that performs an AprilTag-based
 PRECISION LANDING.  PRECISION_LAND now runs a velocity-PD descent in
 GUIDED mode (companion-side, mirroring stationary_landing.py) and only
 commits to ArduCopter LAND mode for the final ~0.6 m of touchdown so the
 autopilot's ground-detection / auto-disarm finishes the landing cleanly.

 Why not pure ArduCopter PrecLand?  A real flight log on this airframe
 showed PrecLand's lateral correction was too weak — body-frame offsets
 persisted around ±1 m through the entire descent and the drone slid
 past the tag laterally while LAND continued at its normal descent rate.
 LANDING_TARGET is still streamed for any PrecLand-aware setup downstream
 but is informational on this airframe; the descent is driven by
 ``PrecisionLandingController.descent_velocity_command``.

 Phase machine
 ─────────────
   INIT → STABILIZE → PATROL ─tag─▶ TRACK ─centred/timeout─▶ PRECISION_LAND
                                       │                          │
                                       │ tag lost                 │ tag lost
                                       ▼                          ▼
                                  IMU RECOVERY              (close-tag? → LAND)
                                  (≤3s body-frame                  │
                                   counter-drift)                  ▼
                                       │                      IMU RECOVERY
                                       │ tag back?               (≤3s)
                                  yes ─┤ ─ no                       │
                                       ▼      ▼              tag back?
                                    TRACK   COMMIT             yes/no
                                            LAND                  │
                                                      ┌───────────┘
                                                      ▼
                                          PRECISION_LAND / COMMIT LAND

   TRACK exits early as soon as both filtered |body_x| and |body_y| are
   below TRACK_CENTER_THRESHOLD_M for TRACK_CENTER_HOLD_FRAMES consecutive
   frames; TRACK_DURATION_S is just an upper bound (currently 20 s).

   Within PRECISION_LAND:
     * Mode is GUIDED while body_z > TOUCHDOWN_BODY_Z_M; control source
       is ``descent_velocity_command`` (companion-side velocity-PD).
     * Mode flips to LAND once body_z < TOUCHDOWN_BODY_Z_M; control
       source is ArduCopter LAND for ground-detection + auto-disarm.

   IMU RECOVERY (new — see RECOVERY_* config) runs whenever the AprilTag
   leaves the camera frame during TRACK or PRECISION_LAND.  It snapshots
   the body-frame drift velocity (Pixhawk EKF, cross-checked against the
   OAK-D S2 onboard BNO086 accelerometer) at the moment of loss and
   commands the opposite velocity for up to RECOVERY_DURATION_S (3 s).
   If the marker reappears we resume the parent phase; if the window
   expires we commit straight to ArduCopter LAND.  The old
   search_and_relocate()/MAX_RESEARCH_ATTEMPTS box-re-fly path is no
   longer triggered from a tag-loss — it is kept in the source as
   reference but unreachable in the current flow.

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
                                                (x/y/z + position_valid=1).
                                                Informational on this airframe —
                                                the actual descent is driven by
                                                ``descent_velocity_command``;
                                                LANDING_TARGET is kept so any
                                                PrecLand-aware setup downstream
                                                still gets the data.
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
TAKEOFF_ALTITUDE  = 5           # meters — matches stationary_landing.py

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
# Search Climb-Back Config — hard altitude cap.
# A prior flight test of the SEARCH phase showed the airframe overshooting
# the requested re-climb altitude by several metres before stabilising
# (the autopilot's position controller could overshoot a goto_ned z target,
# and EKF-origin issues compounded it).  We now velocity-control the climb
# ourselves with a HARD CAP at MAX_SEARCH_ALTITUDE_M.  Defensive coding
# even though the current main flow does not call search_and_relocate
# anymore (the IMU tag-loss recovery commits straight to LAND); if SEARCH
# is ever re-enabled this cap protects against the regression.
# ─────────────────────────────────────────────────────────────────────────────

MAX_SEARCH_ALTITUDE_M       = TAKEOFF_ALTITUDE  # m — hard cap on relative
                                                # altitude during SEARCH-
                                                # climb (matches takeoff
                                                # altitude per user spec).
SEARCH_CLIMB_OVERSHOOT_M    = 0.5   # m — tolerance above the cap before
                                    # we actively command a corrective
                                    # descent.  Below this margin we just
                                    # stop climbing (hover).
SEARCH_CLIMB_VZ             = -0.3  # m/s NED (negative = up).  Mirrors
                                    # the conservative PATROL_SPEED so
                                    # the airframe ascends slowly enough
                                    # that the per-tick altitude check
                                    # can intervene before overshoot.
SEARCH_DESCEND_VZ           = 0.3   # m/s NED (positive = down).  Used
                                    # if rel_alt exceeds the hard cap +
                                    # overshoot margin — descend back
                                    # under the cap before continuing.
SEARCH_CLIMB_TIMEOUT_S      = 20.0  # s — total time budget for the
                                    # safe climb; on exhaustion we hover
                                    # and proceed to the lateral move at
                                    # whatever altitude we achieved.
SEARCH_TARGET_TOLERANCE_M   = 0.3   # m — treat the climb as "complete"
                                    # once we are within this margin of
                                    # the target relative altitude.

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
                                  # descent rather than re-searching.  Kept as
                                  # a SECONDARY fallback only for the case
                                  # where we have no body_z reading at all
                                  # (tag never seen during precision_land);
                                  # the primary close-tag handoff uses
                                  # CLOSE_TAG_BODY_Z_M because EKF z drifts.

CLOSE_TAG_BODY_Z_M     = 1.0      # m — last body-frame z below which a lost
                                  # tag is assumed to be out-of-FOV (we're
                                  # nearly on top of it), not drifted away.
                                  # Preferred over EKF relative altitude for
                                  # the close-tag handoff: real-flight log
                                  # showed EKF z drifting ~3 m so the
                                  # altitude branch fired at alt=-3.37 m,
                                  # technically correct but fragile.

# ── Descent (manual velocity-PD, GUIDED mode) ──────────────────────────────
# When precision-landing on the tag we run a PD controller ourselves rather
# than rely on ArduCopter PrecLand's lateral correction (which on this
# airframe was too weak to keep the camera centred — confirmed in flight
# logs where body-frame offsets persisted around ±1 m through the entire
# descent).  Mirrors the design in
# UAV/PixhawkController/stationary_landing_controller.py.
DESCENT_Kp_XY            = 0.35
DESCENT_Kd_XY            = 0.30  # was 0.25 — extra damping to absorb camera
                                 # pipeline latency (drone keeps moving for a
                                 # frame before the next detection updates).
DESCENT_MAX_V_XY         = 0.4   # m/s, slightly above TRACK for descent
DESCENT_TARGET_BZ        = 0.3   # m, desired height above tag during PD
DESCENT_Kp_Z             = 0.3
DESCENT_MIN_VZ           = 0.10  # m/s minimum descent rate when centred
DESCENT_MAX_VZ           = 0.40  # m/s vertical clamp
DESCENT_XY_ERR_HOLD      = 0.10  # m — beyond this, slow vz to 20 % of cmd.
                                 # Tightened from 0.35 so descent is gated on
                                 # cm-scale lateral alignment, not dm-scale.
DESCENT_DEADBAND_XY      = 0.03  # m — ignore offsets below ~3 cm (was 0.10).
                                 # The user wants single-digit-cm precision so
                                 # the deadband itself must be sub-cm-to-cm.
DESCENT_GAIN_SCALE_BZ    = 1.5   # m — below this, scale XY gains by bz/1.5
DESCENT_MIN_GAIN_SCALE   = 0.30  # floor for the altitude-scaled XY gain
DESCENT_EMA_ALPHA        = 0.65  # heavier filter than TRACK (was 0.5)
DESCENT_STALE_TIMEOUT_S  = 0.5   # re-seed prev_* state after gap longer than this
TOUCHDOWN_BODY_Z_M       = 0.6   # m — switch to LAND below this body-Z, BUT
                                 # only if lateral error is within
                                 # TOUCHDOWN_XY_M (see commit gate in
                                 # precision_land).  If lateral is loose we
                                 # keep doing PD until we either centre or
                                 # sink to TOUCHDOWN_HARD_FLOOR_BZ_M.
TOUCHDOWN_XY_M           = 0.05  # m — max lateral error to permit
                                 # commit-to-LAND.  Single-digit-cm per the
                                 # user spec; honoured as `max(|filt_x|,
                                 # |filt_y|) < TOUCHDOWN_XY_M`.
TOUCHDOWN_HARD_FLOOR_BZ_M = 0.30 # m — below this body-Z, altitude-scaled PD
                                 # gains are too low to centre anyway; commit
                                 # to LAND regardless of lateral error to
                                 # avoid hovering indefinitely on a sub-30cm
                                 # offset we can't drive out.

# ── Camera / pipeline latency ──────────────────────────────────────────────
# The PD loops compute a body-frame offset from an AprilTag detection that
# was captured one camera frame in the past.  Between the capture instant
# and the moment we feed body_x/y into the controller, the airframe has
# already moved along the previously commanded vx/vy.  Two mitigations:
#   1. Skip the PD update entirely if the underlying camera frame is
#      older than CAMERA_STALE_S — neither the measured offset nor a
#      dead-reckon correction can be trusted past that window.
#   2. For frames inside CAMERA_STALE_S, dead-reckon-compensate the body
#      offset:  body_x ← body_x - last_vx * frame_age   (same for y).
#      The sign comes from the body-frame convention: body_x > 0 means
#      tag is forward; commanding vx > 0 moves the drone forward, which
#      reduces body_x at the same rate.
CAMERA_STALE_S           = 0.25  # s — older than this, treat as no detection

# ─────────────────────────────────────────────────────────────────────────────
# TRACK Phase Config — runs between PATROL and PRECISION_LAND.  After the
# patrol acquires the tag, the drone holds station above the tag with a
# velocity-PD loop until either:
#   * filtered |body_x| AND |body_y| stay below TRACK_CENTER_THRESHOLD_M
#     for TRACK_CENTER_HOLD_FRAMES consecutive frames, OR
#   * TRACK_DURATION_S elapses (upper bound).
# This guarantees the descent phase starts from a near-centred hover; in
# an earlier version the loop ran for a fixed 10 s with gains so high the
# velocity command saturated every frame, so the drone twitched at max
# speed without ever converging.
# ─────────────────────────────────────────────────────────────────────────────

TRACK_DURATION_S       = 12.0     # s — upper bound on TRACK; converging
                                  # below TRACK_CENTER_THRESHOLD_M for
                                  # TRACK_CENTER_HOLD_FRAMES exits earlier.
TRACK_LOSS_TIMEOUT_S   = 3.0      # s — bail to SEARCH after this much loss
TRACK_Kp_XY            = 0.35     # P-gain on body-frame position error.
                                  # Raised from 0.22 back to DESCENT's value
                                  # now that track_velocity_command shares
                                  # DESCENT's altitude-scaled gain pathway
                                  # (TRACK_GAIN_SCALE_BZ below):  at cruise
                                  # altitude (>= TRACK_GAIN_SCALE_BZ) the
                                  # scale is 1.0 and Kp matches DESCENT, so
                                  # we converge as fast as DESCENT does; if
                                  # TRACK is ever entered at low altitude
                                  # the scale attenuates Kp the same way the
                                  # descent loop does so we don't twitch.
TRACK_Kd_XY            = 0.30     # D-gain on body-frame position error.
                                  # Slightly above DESCENT_Kd_XY old value
                                  # for extra camera-latency damping.
TRACK_MAX_V_XY         = 0.35     # m/s — per-axis horizontal cap
TRACK_GAIN_SCALE_BZ    = 1.5      # m — mirrors DESCENT_GAIN_SCALE_BZ.  Below
                                  # this body-Z, attenuate XY gains so small
                                  # angular tag-pose errors don't translate
                                  # into a large velocity command.  At TRACK
                                  # altitude (TAKEOFF_ALTITUDE >> 1.5 m) the
                                  # scale evaluates to 1.0.
TRACK_MIN_GAIN_SCALE   = 0.30     # floor for the altitude-scaled XY gain
TRACK_EMA_ALPHA        = 0.6      # was a hard-coded 0.7 inside the function.
                                  # Lowered slightly so the filter lags the
                                  # input less — important because the goal
                                  # of TRACK is precise convergence and a
                                  # heavy filter hides genuine offset moves
                                  # behind the noise floor.  Still well
                                  # above the original 0.5 that chased pose
                                  # noise.

# ── TRACK altitude hold ────────────────────────────────────────────────────
# The previous behaviour was vz = TRACK_VZ_HOLD = 0 (a body-frame velocity
# command with no vertical component).  That tells the autopilot "do not
# command climb or descent" but it does NOT actively HOLD altitude — a
# downdraft / vertical wind just sinks the airframe because GUIDED-mode
# body-frame velocity is open-loop on z.  Flight test confirmed this: the
# drone steadily descended throughout TRACK in moderate wind.
#
# We now close the loop on altitude using the relative altitude reported
# against takeoff_z_origin (same anchor the rest of the script uses).
# Target = TRACK_TARGET_ALT_M (TAKEOFF_ALTITUDE per the user spec — the
# "set height").  Each tick:  vz = -TRACK_Kp_Z * (target - rel_alt),
# deadbanded by TRACK_ALT_DEADBAND_M and clamped to ±TRACK_MAX_VZ.
# Sign reminder: NED z is down-positive, so vz < 0 commands a climb.
TRACK_TARGET_ALT_M     = TAKEOFF_ALTITUDE  # m — TRACK actively holds this
                                           # relative altitude (above
                                           # takeoff_z_origin).
TRACK_Kp_Z             = 0.5      # gain on altitude error.  0.5 m/s per
                                  # m of error means a full-scale
                                  # TRACK_MAX_VZ correction at ~0.8 m
                                  # error; intermediate errors get
                                  # proportional response.
TRACK_MAX_VZ           = 0.40     # m/s — per-axis vertical clamp during
                                  # TRACK.  Kept moderate so a brief
                                  # altitude excursion doesn't trigger
                                  # an aggressive climb/descend that
                                  # would also kick the tag out of FOV.
TRACK_ALT_DEADBAND_M   = 0.10     # m — ignore altitude errors below this
                                  # so EKF z noise doesn't drive a
                                  # constant tiny vz command.
TRACK_DEADBAND_XY      = 0.03     # m — ignore offsets below ~3 cm.  Was
                                  # 0.08; tightened to single-digit cm so
                                  # the controller still drives the drone
                                  # at small errors.  Cap noise from pose
                                  # estimation is mostly in the sub-cm
                                  # range at TRACK altitudes.
TRACK_CENTER_THRESHOLD_M = 0.05   # m — both filtered |body_x| and |body_y|
                                  # must drop below this for TRACK to
                                  # consider itself "centred".  Tightened
                                  # from 0.25 m to 5 cm per the user spec
                                  # (single-digit-cm precision).
TRACK_CENTER_HOLD_FRAMES = 15     # consecutive ticks the centred condition
                                  # must hold before TRACK exits early.
                                  # Raised from 8 to compensate for the
                                  # tighter threshold — at the 20 Hz TRACK
                                  # loop rate (sleep 0.05 s) this is 0.75 s
                                  # of sustained convergence, robust to a
                                  # single noisy detection that briefly
                                  # creeps above 5 cm.

# ─────────────────────────────────────────────────────────────────────────────
# IMU Tag-Loss Recovery Config
# ─────────────────────────────────────────────────────────────────────────────
# When the AprilTag falls out of the camera frame mid-flight (most likely
# cause: a wind gust pushing the airframe laterally), we no longer just hover
# during the loss window — that lets the drift continue and almost always
# ends in SEARCH or a bad commit-to-LAND off-target.
#
# Instead, at the moment of loss we capture a body-frame "drift snapshot"
# from two independent IMU-driven signals:
#
#   PRIMARY  — Pixhawk LOCAL_POSITION_NED velocity (vx, vy) rotated into
#              body frame using ATTITUDE.yaw.  These vx/vy are the EKF's
#              fused estimate, driven primarily by the FCU IMU between
#              GPS updates, so they are the most accurate body-frame
#              velocity reading available on this airframe.
#
#   BACKUP   — OAK-D S2 onboard BNO086 accelerometer (ACCELEROMETER_RAW),
#              mapped from camera frame to body frame with the existing
#              camera_to_body() transform.  Used purely as a confidence
#              cross-check: if the OAK IMU also sees lateral acceleration
#              above RECOVERY_OAK_ACCEL_MIN, the Pixhawk-derived drift
#              estimate is trusted at full gain; otherwise the counter-
#              command is attenuated to RECOVERY_OAK_DISAGREE_SCALE.
#
# During the loss window (RECOVERY_DURATION_S — matches the existing
# TAG_LOSS_TIMEOUT / TRACK_LOSS_TIMEOUT_S so we replace rather than
# extend), we command a constant body-frame counter-velocity equal to
# ``-RECOVERY_KP * drift_snapshot``.  Using a SNAPSHOT (not a closed-loop
# feedback on current velocity) is deliberate: once the drone decelerates
# and stops, instantaneous velocity → 0 but we still need to keep flying
# back toward the marker, so the command must persist.  Magnitude is
# clamped to RECOVERY_MAX_V_XY and floored at RECOVERY_MIN_V_XY whenever
# the detected drift is above RECOVERY_DRIFT_DEADBAND.
#
# If the marker re-appears before the 3 s window expires we exit recovery
# and resume the parent phase (TRACK or PRECISION_LAND).  If the window
# expires without re-acquisition we commit directly to ArduCopter LAND
# (NOT search_and_relocate) per the user spec.

RECOVERY_DURATION_S        = 3.0   # s — must match TAG_LOSS_TIMEOUT and
                                   # TRACK_LOSS_TIMEOUT_S; we are REPLACING
                                   # the old static-hover loss window, not
                                   # extending it.
RECOVERY_KP                = 1.2   # gain on the counter-drift velocity
                                   # (v_cmd_body = -RECOVERY_KP * drift_body)
RECOVERY_MAX_V_XY          = 0.4   # m/s — per-axis clamp on counter command
RECOVERY_MIN_V_XY          = 0.10  # m/s — minimum magnitude per axis when
                                   # drift on that axis is above the
                                   # deadband; ensures a tiny but real
                                   # drift still produces real motion
                                   # rather than collapsing under the clamp
RECOVERY_DRIFT_DEADBAND    = 0.05  # m/s — body-frame drift below this on
                                   # both axes is treated as noise (no
                                   # counter command on that axis)
RECOVERY_OAK_ACCEL_MIN     = 0.30  # m/s² — minimum OAK-D lateral-accel
                                   # magnitude (XY in body frame, gravity
                                   # is on body-Z for a downward camera so
                                   # XY is gravity-free to first order) to
                                   # count as "the camera IMU sees motion"
RECOVERY_OAK_DISAGREE_SCALE = 0.6  # gain scale when the OAK IMU does NOT
                                   # confirm motion; we still apply the
                                   # Pixhawk-derived counter command but
                                   # at reduced authority
RECOVERY_OAK_EMA_ALPHA     = 0.7   # EMA on OAK accel samples in the pump
                                   # (heavy filter — BNO086 raw is noisy
                                   # at the 100 Hz pipeline rate)
RECOVERY_OAK_IMU_HZ        = 100   # OAK IMU sample rate for both the
                                   # accelerometer and gyroscope streams
RECOVERY_FALLBACK_OFFSET_M = 0.20  # m — if BOTH IMU sources show drift
                                   # below their deadbands at the moment
                                   # of loss, fall back to the last
                                   # body-frame TAG OFFSET as the drift
                                   # direction (drone is on the opposite
                                   # side of the marker by definition).
                                   # Below this body-frame offset we give
                                   # up on direction inference and hover.

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

        # ATTITUDE cache — needed by body_frame_velocity() to rotate the
        # NED velocity into the airframe body frame for the IMU tag-loss
        # recovery.  Yaw alone is sufficient for that rotation; we cache
        # roll/pitch too so the HUD or future logic can use them.
        self.last_att = {
            "roll": None, "pitch": None, "yaw": None,
            "t": 0.0,
        }

        # Pixhawk SCALED_IMU cache — body-frame accelerometer in m/s².
        # SCALED_IMU reports accelerations in mG (milli-g); we convert
        # to m/s² on intake so downstream consumers don't have to.
        self.last_pix_imu = {
            "ax": None, "ay": None, "az": None,
            "gx": None, "gy": None, "gz": None,
            "t": 0.0,
        }

        # OAK-D S2 (BNO086) IMU cache — written by make_pump() as
        # accelerometer samples arrive on the DepthAI queue.  Stored in
        # the AIRFRAME body frame (already passed through camera_to_body)
        # so callers don't need to know about the camera convention.
        # ema_ax/ay/az are EMA-smoothed (alpha = RECOVERY_OAK_EMA_ALPHA)
        # to suppress per-sample noise from the 100 Hz raw stream.
        self.last_oak_imu = {
            "ax": None, "ay": None, "az": None,
            "ema_ax": None, "ema_ay": None, "ema_az": None,
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

        # DESCENT-phase prev state — separate from TRACK so re-entering
        # PRECISION_LAND after a SEARCH does not carry stale TRACK history.
        # filt_z is filtered alongside body_z so the vertical command is
        # computed against the same smoothed signal as the gain-scale.
        self._descent_prev_x = None
        self._descent_prev_y = None
        self._descent_prev_z = None
        self._descent_prev_t = None

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
        """Ask the FCU to stream the messages we rely on at known rates.

        ArduCopter typically sends LOCAL_POSITION_NED / ATTITUDE / SCALED_IMU
        by default, but on some configurations it does not — the takeoff
        and stabilization checks rely on LOCAL_POSITION_NED, and the new
        IMU tag-loss recovery (recover_velocity_command) relies on
        ATTITUDE (for the NED→body yaw rotation) and SCALED_IMU (as a
        confidence cross-check alongside the OAK-D IMU).  A missing
        stream silently breaks any of those paths, so request each
        explicitly at a useful rate.

        Mirrors the call pattern the existing
        TestComponents/test_*_local_position.py scripts make.
        """
        streams = [
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10, "LOCAL_POSITION_NED"),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,           20, "ATTITUDE"),
            (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,         20, "SCALED_IMU"),
        ]
        for msg_id, hz, name in streams:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    1e6 / hz,    # interval in microseconds
                    0, 0, 0, 0, 0,
                )
                print(f"[INFO] Requested {name} @ {hz} Hz")
            except Exception as e:
                print(f"[WARN] Could not request {name} stream: {e}")

    def _drain_messages(self):
        """Drain the MAVLink buffer non-blockingly, caching latest telemetry.

        Must be called inside any hot loop — without it the serial link
        eventually overflows and the FCU starts dropping inbound commands.
        The existing scripts call recv_match(blocking=False) for the same
        reason; this just centralises it and stashes useful fields.
        """
        # Bumped from 20 → 40 because three streams are now active
        # (LOCAL_POSITION_NED @10 Hz + ATTITUDE @20 Hz + SCALED_IMU @20 Hz);
        # at the old cap a single tick could leave SCALED_IMU stale even
        # when fresh samples were pending on the buffer.
        for _ in range(40):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            mtype = msg.get_type()
            if mtype == "LOCAL_POSITION_NED":
                self.last_pos["x"]  = msg.x
                self.last_pos["y"]  = msg.y
                self.last_pos["z"]  = msg.z
                self.last_pos["vx"] = msg.vx
                self.last_pos["vy"] = msg.vy
                self.last_pos["vz"] = msg.vz
                self.last_pos["t"]  = time.time()
            elif mtype == "ATTITUDE":
                # ATTITUDE fields are already in radians; yaw is the
                # heading angle of the body X axis (forward) measured
                # CW from North in NED.  Used directly by
                # body_frame_velocity() to rotate vx/vy into body frame.
                self.last_att["roll"]  = msg.roll
                self.last_att["pitch"] = msg.pitch
                self.last_att["yaw"]   = msg.yaw
                self.last_att["t"]     = time.time()
            elif mtype == "SCALED_IMU":
                # SCALED_IMU.xacc/yacc/zacc are in mG (milli-g, int16);
                # convert to m/s² here so callers never have to remember
                # the unit.  gyros are mrad/s.
                self.last_pix_imu["ax"] = msg.xacc * 9.80665 / 1000.0
                self.last_pix_imu["ay"] = msg.yacc * 9.80665 / 1000.0
                self.last_pix_imu["az"] = msg.zacc * 9.80665 / 1000.0
                self.last_pix_imu["gx"] = msg.xgyro / 1000.0
                self.last_pix_imu["gy"] = msg.ygyro / 1000.0
                self.last_pix_imu["gz"] = msg.zgyro / 1000.0
                self.last_pix_imu["t"]  = time.time()

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

    def track_velocity_command(self, body_x, body_y, body_z, frame_age=0.0):
        """Drive the drone to (0, 0) in body-frame XY using EMA + PD,
        while actively holding altitude at TRACK_TARGET_ALT_M.

        Used by the TRACK phase between PATROL and PRECISION_LAND so the
        drone is centered above the tag BEFORE any descent begins.

        Now structurally aligned with descent_velocity_command:
            filter → deadband → derivative → altitude-scaled PD → clamp
            → send_velocity.
        body_z is the camera-to-tag distance (m) and feeds the same
        gain-scale used in descent_velocity_command:  if body_z is
        below TRACK_GAIN_SCALE_BZ, XY gains and clamp are attenuated
        so small angular pose errors don't translate into a large
        velocity command.  At TRACK altitude this evaluates to scale=1.0
        and the loop behaves like descent's XY PD at cruise.

        frame_age (s) is the wall-clock age of the AprilTag detection
        (now - frame-capture-time).  We dead-reckon-correct the input
        body offset by the previously commanded velocity:

            body_x ← body_x - last_vx * frame_age
            body_y ← body_y - last_vy * frame_age

        The sign comes from the body-frame convention: body_x > 0 means
        the tag is forward; commanding vx > 0 moves the drone forward,
        which shrinks body_x at the same rate.  This compensates for the
        camera-to-control pipeline latency so the controller acts on
        "where the tag is NOW" rather than "where the tag was when the
        last frame was captured".

        Vertical control: the previous version commanded vz = 0 as an
        "altitude hold", but body-frame velocity is open-loop on z in
        ArduCopter GUIDED — a downdraft just sinks the airframe.  A
        flight test confirmed this: the drone steadily descended during
        TRACK in moderate wind.  We now run a P-loop on the relative
        altitude (anchored to takeoff_z_origin, same convention as the
        rest of the script):

            alt_err = TRACK_TARGET_ALT_M - rel_alt   (+ = below target)
            vz_cmd  = -TRACK_Kp_Z * alt_err          (NED: -vz = climb)
            vz      = clamp(vz_cmd, ±TRACK_MAX_VZ)

        If either LOCAL_POSITION_NED or the takeoff anchor is missing
        (degenerate startup), vz falls back to 0 — same behaviour as
        the old TRACK_VZ_HOLD code so we never make things worse.

        Filter / derivative state for XY mirrors
        stationary_landing_controller.adjust_velocity_and_send():
          * Re-seed prev state if it is None or older than 0.5 s.  A
            stale buffer would produce a huge spurious D-term spike on
            the next valid frame.
          * Apply TRACK_EMA_ALPHA EMA on the body-frame inputs (0.6 —
            slightly less smoothing than the previous hard-coded 0.7 so
            the controller responds faster, important now that the
            convergence threshold is 5 cm not 25 cm).
          * Compute the derivative on the FILTERED values.
          * Apply a TRACK_DEADBAND_XY (~3 cm) deadband to the filtered
            error so AprilTag pose noise doesn't drive a constant
            sub-cm velocity command.

        Returns the (filt_x, filt_y, vx, vy, vz) actually sent so the
        caller can both log the command and use the filtered offsets to
        decide TRACK_CENTER_THRESHOLD_M convergence.
        """
        now = time.time()

        # Latency lead/dead-reckon: the camera frame this body offset came
        # from was captured frame_age seconds ago.  In the meantime we
        # have been commanding (last_vx, last_vy) for that same window,
        # so the offset relative to NOW is approximately:
        #     body_x_now ≈ body_x_measured - last_vx * frame_age
        # The same sign for body_y.  No correction when frame_age == 0.
        if frame_age > 0.0:
            body_x -= self.last_cmd.get("vx", 0.0) * frame_age
            body_y -= self.last_cmd.get("vy", 0.0) * frame_age

        if (self._track_prev_x is None
                or self._track_prev_t is None
                or (now - self._track_prev_t) > 0.5):
            self._track_prev_x = body_x
            self._track_prev_y = body_y
            self._track_prev_t = now

        alpha = TRACK_EMA_ALPHA
        filt_x = alpha * self._track_prev_x + (1 - alpha) * body_x
        filt_y = alpha * self._track_prev_y + (1 - alpha) * body_y

        # Clamp dt to suppress div-by-zero on the seeded frame and
        # D-term spikes on frame skips.
        dt = max(min(now - self._track_prev_t, 0.2), 0.01)
        dx = (filt_x - self._track_prev_x) / dt
        dy = (filt_y - self._track_prev_y) / dt

        # Deadband on the FILTERED error so AprilTag pose noise inside
        # ±TRACK_DEADBAND_XY doesn't keep nudging the drone around.
        err_x = 0.0 if abs(filt_x) < TRACK_DEADBAND_XY else filt_x
        err_y = 0.0 if abs(filt_y) < TRACK_DEADBAND_XY else filt_y

        # Altitude-aware gain scaling mirrors descent_velocity_command.
        # At TRACK cruise altitude (body_z >= TRACK_GAIN_SCALE_BZ) scale
        # is 1.0 so we run at full gain; only matters if TRACK is ever
        # re-entered close to the tag.
        if body_z < TRACK_GAIN_SCALE_BZ:
            scale = max(TRACK_MIN_GAIN_SCALE,
                        body_z / TRACK_GAIN_SCALE_BZ)
        else:
            scale = 1.0

        vx = scale * (TRACK_Kp_XY * err_x + TRACK_Kd_XY * dx)
        vy = scale * (TRACK_Kp_XY * err_y + TRACK_Kd_XY * dy)

        xy_cap = scale * TRACK_MAX_V_XY
        vx = max(min(vx, xy_cap), -xy_cap)
        vy = max(min(vy, xy_cap), -xy_cap)

        # ── Active altitude hold ─────────────────────────────────────────
        # Compute from the same anchored-relative altitude as the rest
        # of the script (_relative_altitude_m).  Bail-out path keeps
        # vz = 0 if we have no altitude reading, so the airframe is at
        # worst no worse off than the previous TRACK_VZ_HOLD behaviour.
        rel_alt = _relative_altitude_m(self)
        if rel_alt is None:
            vz = 0.0
        else:
            alt_err = TRACK_TARGET_ALT_M - rel_alt
            if abs(alt_err) < TRACK_ALT_DEADBAND_M:
                vz = 0.0
            else:
                vz_cmd = -TRACK_Kp_Z * alt_err
                vz = max(min(vz_cmd, TRACK_MAX_VZ), -TRACK_MAX_VZ)

        self._track_prev_x = filt_x
        self._track_prev_y = filt_y
        self._track_prev_t = now

        self.send_velocity(vx, vy, vz)
        return filt_x, filt_y, vx, vy, vz

    # ── PRECISION_LAND phase: velocity-PD descent ────────────────────────────

    def descent_velocity_command(self, body_x, body_y, body_z, frame_age=0.0):
        """Velocity-PD descent that keeps the camera centred over the tag.

        Replaces ArduCopter PrecLand's lateral correction (which on this
        airframe was too weak to centre the descent — flight log showed
        body-frame offsets persisting around ±1 m all the way down).
        Mirrors the design in
        UAV/PixhawkController/stationary_landing_controller.adjust_velocity_and_send:
        altitude-aware gain scheduling, descent rate coupled to XY error,
        EMA smoothing.

        Important behaviour vs track_velocity_command:
          * Commands ``vz`` (descent) — TRACK keeps vz tied to altitude.
          * Heavier EMA (alpha = DESCENT_EMA_ALPHA, 0.65 vs TRACK's 0.6).
          * XY gains are scaled DOWN as we get close to the tag so the
            drone doesn't over-react when small pixel errors translate
            to small physical errors.
          * Vertical command is throttled to 20 % whenever the lateral
            error is larger than DESCENT_XY_ERR_HOLD (now 10 cm) —
            recentre first, then descend.
          * Never commands upward velocity; if filt_z drops below
            DESCENT_TARGET_BZ the caller is expected to commit to LAND
            for the final touchdown.

        ``frame_age`` (s) is the wall-clock age of the AprilTag detection
        used for ``body_x/body_y``.  We dead-reckon-correct by subtracting
        ``last_vx * frame_age`` (resp. ``last_vy``) from the measured
        offset before filtering — same rationale as
        ``track_velocity_command``.  No correction when frame_age is 0.

        Returns the (filt_x, filt_y, vx, vy, vz) actually sent so the
        precision_land driver can decide whether the lateral error is
        small enough to commit to LAND without separately re-computing
        the same EMA + deadband logic.
        """
        now = time.time()

        # Latency lead/dead-reckon: see track_velocity_command for the
        # full derivation.  Applied before the EMA so the filter
        # smooths the *corrected* signal, not the raw one.
        if frame_age > 0.0:
            body_x -= self.last_cmd.get("vx", 0.0) * frame_age
            body_y -= self.last_cmd.get("vy", 0.0) * frame_age

        if (self._descent_prev_x is None
                or self._descent_prev_t is None
                or (now - self._descent_prev_t) > DESCENT_STALE_TIMEOUT_S):
            self._descent_prev_x = body_x
            self._descent_prev_y = body_y
            self._descent_prev_z = body_z
            self._descent_prev_t = now

        a = DESCENT_EMA_ALPHA
        filt_x = a * self._descent_prev_x + (1 - a) * body_x
        filt_y = a * self._descent_prev_y + (1 - a) * body_y
        filt_z = a * self._descent_prev_z + (1 - a) * body_z

        dt = max(min(now - self._descent_prev_t, 0.2), 0.01)
        dx = (filt_x - self._descent_prev_x) / dt
        dy = (filt_y - self._descent_prev_y) / dt

        # Deadband on filtered XY error.
        err_x = 0.0 if abs(filt_x) < DESCENT_DEADBAND_XY else filt_x
        err_y = 0.0 if abs(filt_y) < DESCENT_DEADBAND_XY else filt_y

        # Altitude-aware gain scaling: when we're closer than
        # DESCENT_GAIN_SCALE_BZ to the tag, attenuate XY gains so a fixed
        # angular tag-pose error doesn't translate into a too-large
        # lateral velocity command.
        if filt_z < DESCENT_GAIN_SCALE_BZ:
            scale = max(DESCENT_MIN_GAIN_SCALE,
                        filt_z / DESCENT_GAIN_SCALE_BZ)
        else:
            scale = 1.0

        vx = scale * (DESCENT_Kp_XY * err_x + DESCENT_Kd_XY * dx)
        vy = scale * (DESCENT_Kp_XY * err_y + DESCENT_Kd_XY * dy)

        xy_cap = scale * DESCENT_MAX_V_XY
        vx = max(min(vx, xy_cap), -xy_cap)
        vy = max(min(vy, xy_cap), -xy_cap)

        # Vertical command: descend toward DESCENT_TARGET_BZ.
        error_z = filt_z - DESCENT_TARGET_BZ
        if error_z > 0.0:
            vz_cmd = max(DESCENT_MIN_VZ, DESCENT_Kp_Z * error_z)
            # Slow vz when XY error is still meaningful — recentre first.
            if max(abs(filt_x), abs(filt_y)) > DESCENT_XY_ERR_HOLD:
                vz_cmd *= 0.2
            vz = min(vz_cmd, DESCENT_MAX_VZ)
        else:
            # Below target altitude — let caller commit to LAND.  Never
            # command upward velocity here; we'd just chase noise.
            vz = 0.0

        self._descent_prev_x = filt_x
        self._descent_prev_y = filt_y
        self._descent_prev_z = filt_z
        self._descent_prev_t = now

        self.send_velocity(vx, vy, vz)
        # The HUD's LT branch in draw_overlay() should fall back to the
        # velocity branch during DESCENT — clear the LANDING_TARGET cache
        # so the operator sees the velocity command driving the airframe.
        self.last_cmd["lt_x"] = None
        self.last_cmd["lt_y"] = None
        self.last_cmd["lt_z"] = None
        return filt_x, filt_y, vx, vy, vz

    # ── IMU tag-loss recovery (body-frame counter-drift) ─────────────────────

    def body_frame_velocity(self):
        """Return the airframe's body-frame velocity (vx, vy, vz) in m/s.

        Computed by rotating the EKF's NED velocity (cached in
        ``last_pos``) by the negative of the body yaw (cached in
        ``last_att``).  Used by the IMU tag-loss recovery to determine
        which direction the airframe is drifting in its OWN frame —
        which is what the velocity-control surface
        (``send_velocity`` / MAV_FRAME_BODY_NED) speaks.

        The standard NED → body rotation is:

            [body_x]   [ cos(yaw)   sin(yaw)  0 ] [vN]
            [body_y] = [-sin(yaw)   cos(yaw)  0 ] [vE]
            [body_z]   [   0          0       1 ] [vD]

        where yaw is the ArduPilot ATTITUDE.yaw — heading of body-X CW
        from North in NED, radians.

        Returns (None, None, None) if either the velocity or the yaw
        has not yet been received.  Callers MUST guard against that —
        the recovery should fall back to its other signals (OAK IMU,
        last tag offset) when this returns None.
        """
        self._drain_messages()
        vx = self.last_pos.get("vx")
        vy = self.last_pos.get("vy")
        vz = self.last_pos.get("vz")
        yaw = self.last_att.get("yaw")
        if vx is None or vy is None or yaw is None:
            return None, None, None
        c = math.cos(yaw)
        s = math.sin(yaw)
        body_vx =  vx * c + vy * s
        body_vy = -vx * s + vy * c
        body_vz = vz if vz is not None else 0.0
        return body_vx, body_vy, body_vz

    def recover_velocity_command(self, drift_snapshot, last_tag_body=None):
        """Command a body-frame counter-drift velocity to fly back toward
        a marker that has just left the camera frame, AND actively
        return to the set altitude (TRACK_TARGET_ALT_M) while doing so.

        ``drift_snapshot`` is the (body_vx, body_vy) sampled at the
        moment of tag loss (NOT the current instantaneous velocity —
        see the rationale in the module-level IMU Tag-Loss Recovery
        Config comment).  ``last_tag_body`` is the (body_x, body_y)
        offset of the marker on the LAST frame it was visible — used
        as a fallback drift direction when both the Pixhawk-derived
        velocity AND the OAK-D IMU show too little motion to be
        trustworthy (e.g. a perfectly hovering drone hit by a sudden
        gust just as the marker drifted past the FOV edge).

        Cross-checks the snapshot against the OAK-D BNO086 lateral
        acceleration cached in ``last_oak_imu`` — if the OAK IMU also
        sees lateral motion above RECOVERY_OAK_ACCEL_MIN we trust the
        snapshot at full gain, otherwise we attenuate the command to
        RECOVERY_OAK_DISAGREE_SCALE.

        Vertical: per user spec the recovery also flies the airframe
        back UP to TRACK_TARGET_ALT_M (the takeoff "set height", 4 m
        on this airframe).  Reuses the TRACK altitude-hold P-loop
        (TRACK_Kp_Z / TRACK_MAX_VZ / TRACK_ALT_DEADBAND_M) so the
        airframe converges on the same altitude regardless of
        whether the marker is currently visible.  Important during a
        tag-loss in PRECISION_LAND: the recovery actively climbs back
        from whatever descent altitude we were at, giving the camera
        more vertical headroom to re-acquire the marker.

        Sends a body-frame velocity (vx, vy, vz) and returns a dict
        describing the action so the caller can log it / surface it on
        the HUD:

            {
                "vx": float, "vy": float, "vz": float,
                "drift_x": float, "drift_y": float,
                "rel_alt": float | None,
                "source": "ekf" | "tag_offset" | "none",
                "oak_confirm": bool,
                "scale": float,
            }
        """
        self._drain_messages()

        snap_x, snap_y = drift_snapshot
        source = "ekf"

        # If the Pixhawk-derived snapshot is below the deadband on both
        # axes, fall back to the last-tag-offset direction.  A marker
        # that was at body_x > 0 (in front of the drone) when last seen
        # implies the drone is BEHIND the marker on the +X side from
        # the marker's perspective — so to bring the marker back into
        # FOV, we want to move IN the direction of the last offset
        # (chase the marker), not opposite.  Wait, that's wrong: the
        # marker offset is from the drone's POV, so positive body_x
        # means the marker is forward of the drone; if it just left
        # the FOV, the drone needs to move FORWARD (+body_x) to recover
        # it.  So the fallback "drift" we want to counteract is the
        # NEGATIVE of the last offset — i.e. the drone "drifted away"
        # in the direction OPPOSITE to where the marker is.
        snap_mag = max(abs(snap_x), abs(snap_y))
        if (snap_mag < RECOVERY_DRIFT_DEADBAND
                and last_tag_body is not None):
            off_x, off_y = last_tag_body
            if (abs(off_x) >= RECOVERY_FALLBACK_OFFSET_M
                    or abs(off_y) >= RECOVERY_FALLBACK_OFFSET_M):
                # Treat the last offset as drift in the OPPOSITE
                # direction (drone drifted away from where marker is),
                # so the counter command will be IN the direction of
                # the marker.  See sign reasoning above.
                snap_x = -off_x
                snap_y = -off_y
                source = "tag_offset"
            else:
                source = "none"

        # OAK-D cross-check: only meaningful if we actually have an EMA
        # sample.  The OAK accel is body-frame XY (gravity is on body-Z
        # for a downward camera and rejected here by ignoring az).
        oak_ax = self.last_oak_imu.get("ema_ax")
        oak_ay = self.last_oak_imu.get("ema_ay")
        oak_confirm = False
        if oak_ax is not None and oak_ay is not None:
            oak_mag = math.sqrt(oak_ax * oak_ax + oak_ay * oak_ay)
            oak_confirm = oak_mag >= RECOVERY_OAK_ACCEL_MIN

        # Apply confidence scaling.  When the EKF snapshot itself was
        # degenerate AND we fell through to "none", we don't have a
        # direction to command — just hover.
        if source == "none":
            scale = 0.0
        else:
            scale = 1.0 if oak_confirm else RECOVERY_OAK_DISAGREE_SCALE

        vx_raw = -RECOVERY_KP * snap_x * scale
        vy_raw = -RECOVERY_KP * snap_y * scale

        # Per-axis minimum: if the axis drift is above its deadband and
        # we have a direction, ensure the command magnitude is at least
        # RECOVERY_MIN_V_XY so the drone visibly moves instead of
        # collapsing under the clamp.  Sign is preserved.
        def _floor_then_clamp(v, drift_on_axis):
            if abs(drift_on_axis) < RECOVERY_DRIFT_DEADBAND or scale == 0.0:
                return 0.0
            if abs(v) < RECOVERY_MIN_V_XY:
                v = math.copysign(RECOVERY_MIN_V_XY, v if v != 0 else -drift_on_axis)
            return max(min(v, RECOVERY_MAX_V_XY), -RECOVERY_MAX_V_XY)

        vx = _floor_then_clamp(vx_raw, snap_x)
        vy = _floor_then_clamp(vy_raw, snap_y)

        # ── Active altitude hold during recovery ─────────────────────────
        # Identical P-loop to track_velocity_command — bring the airframe
        # back to TRACK_TARGET_ALT_M (the takeoff set-height) regardless
        # of what altitude the tag was lost at.  Fall back to vz=0 if
        # the relative-altitude reading is not available (degenerate
        # startup), same safe behaviour as TRACK.
        rel_alt = _relative_altitude_m(self)
        if rel_alt is None:
            vz = 0.0
        else:
            alt_err = TRACK_TARGET_ALT_M - rel_alt
            if abs(alt_err) < TRACK_ALT_DEADBAND_M:
                vz = 0.0
            else:
                vz_cmd = -TRACK_Kp_Z * alt_err
                vz = max(min(vz_cmd, TRACK_MAX_VZ), -TRACK_MAX_VZ)

        self.send_velocity(vx, vy, vz)

        # Same HUD-cache convention as descent_velocity_command — clear
        # the LANDING_TARGET fields so draw_overlay falls through to
        # the velocity branch (which is what's actually driving the
        # motors during recovery).
        self.last_cmd["lt_x"] = None
        self.last_cmd["lt_y"] = None
        self.last_cmd["lt_z"] = None

        return {
            "vx": vx, "vy": vy, "vz": vz,
            "drift_x": snap_x, "drift_y": snap_y,
            "rel_alt": rel_alt,
            "source": source,
            "oak_confirm": oak_confirm,
            "scale": scale,
        }

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
        # When the IMU recovery is active during the tag-loss window,
        # leg_label already carries "TRACK RECOVER..." or "PL RECOVER..."
        # — surface that on the TAG line too so the operator immediately
        # sees the recovery is in progress (instead of just "LOST").
        if "RECOVER" in leg_label:
            lines.append(
                (f"TAG     LOST ({time_lost:.1f}s) — IMU RECOVER", color_tag)
            )
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

def make_pump(q_rgb, q_oak_imu, detector, controller, state):
    """Build a closure that pulls the latest frame, detects (if requested),
    refreshes the overlay HUD, and pumps cv2.waitKey so the window stays live.

    Returns a zero-arg function suitable for handing to long-blocking
    controller methods (takeoff_to_altitude, wait_stabilized, goto_ned)
    so the camera does not freeze during those phases.

    ``q_oak_imu`` is the DepthAI queue for OAK-D S2 accelerometer samples
    (ACCELEROMETER_RAW @ RECOVERY_OAK_IMU_HZ).  Drained every pump tick and
    EMA-smoothed into ``controller.last_oak_imu`` for use by the IMU
    tag-loss recovery (recover_velocity_command).  The transform from the
    OAK-D camera frame to the airframe body frame uses the existing
    PrecisionLandingController.camera_to_body() static method — same
    convention as the AprilTag pose path.

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

        # 1b. Drain OAK-D IMU samples and EMA-smooth into the controller
        #     cache so recover_velocity_command() has a fresh body-frame
        #     accel reading available the instant the tag is lost.  We
        #     drain ALL pending packets each tick (not just the latest)
        #     so the EMA reflects the trajectory of the last ~tick
        #     window rather than a single point.
        if q_oak_imu is not None:
            try:
                imu_msgs = q_oak_imu.tryGetAll()
            except Exception:
                imu_msgs = []
            alpha = RECOVERY_OAK_EMA_ALPHA
            for msg in imu_msgs:
                for pkt in getattr(msg, "packets", []):
                    accel = getattr(pkt, "acceleroMeter", None)
                    if accel is None:
                        accel = getattr(pkt, "accelerometer", None)
                    if accel is None:
                        continue
                    cam_ax = float(accel.x)
                    cam_ay = float(accel.y)
                    cam_az = float(accel.z)
                    # camera_to_body assumes the same downward-camera
                    # convention as the AprilTag pose path.  IMU axes on
                    # OAK-D generally track the camera optical axes, so
                    # the same transform applies — flagged here so the
                    # mounting convention is visible to future readers.
                    body_ax, body_ay, body_az = \
                        PrecisionLandingController.camera_to_body(
                            cam_ax, cam_ay, cam_az,
                        )
                    controller.last_oak_imu["ax"] = body_ax
                    controller.last_oak_imu["ay"] = body_ay
                    controller.last_oak_imu["az"] = body_az
                    prev_ax = controller.last_oak_imu["ema_ax"]
                    prev_ay = controller.last_oak_imu["ema_ay"]
                    prev_az = controller.last_oak_imu["ema_az"]
                    if prev_ax is None:
                        controller.last_oak_imu["ema_ax"] = body_ax
                        controller.last_oak_imu["ema_ay"] = body_ay
                        controller.last_oak_imu["ema_az"] = body_az
                    else:
                        controller.last_oak_imu["ema_ax"] = (
                            alpha * prev_ax + (1 - alpha) * body_ax)
                        controller.last_oak_imu["ema_ay"] = (
                            alpha * prev_ay + (1 - alpha) * body_ay)
                        controller.last_oak_imu["ema_az"] = (
                            alpha * prev_az + (1 - alpha) * body_az)
                    controller.last_oak_imu["t"] = time.time()

        # 2. Pull the most recent camera frame (if any).  tryGet() never
        #    blocks; if the queue is empty we render the last cached frame
        #    so the overlay still updates.  state["last_frame_time"] is
        #    refreshed ONLY when a genuinely new frame arrives — used by
        #    the PD loops as a freshness signal so they can skip / dead-
        #    reckon-correct on stale cached frames.
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            state["frame"] = in_rgb.getCvFrame()
            state["last_frame_time"] = time.time()

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
# Commit-to-LAND helper — used by both the close-tag handoff in
# precision_land() AND the new IMU-recovery timeout in track_tag() /
# precision_land().  Lifted to module scope so callers in either phase can
# share it; previously it was a closure inside precision_land().
# ─────────────────────────────────────────────────────────────────────────────

def commit_to_land(controller, pump, reason):
    """Switch to ArduCopter LAND mode and block until motors disarm.

    LAND handles ground-detection + auto-disarm, which is exactly what
    we want for the final touchdown.  We sit in a 30 s pump loop so the
    HUD keeps updating during the descent.
    """
    print(f"[INFO] {reason} — committing to LAND descent until touchdown")
    controller.change_flight_mode("LAND")
    land_start = time.time()
    while time.time() - land_start < 30:
        controller._drain_messages()
        if not controller.master.motors_armed():
            print("[INFO] Motors disarmed — touchdown.")
            return
        pump()
        time.sleep(0.1)


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
# Tag-loss handling within TRACK runs the IMU recovery (see
# recover_velocity_command) — we capture the body-frame drift velocity
# the instant the marker leaves the FOV and command the opposite
# direction for up to RECOVERY_DURATION_S.  If the marker re-appears
# we resume tracking; if the window expires we commit directly to
# LAND (the user's spec; SEARCH is no longer triggered from TRACK).
#
# Returns:
#   "READY"        — duration elapsed, hand off to PRECISION_LAND
#   "COMMIT_LAND"  — IMU recovery window expired without re-acquisition;
#                    caller commits to ArduCopter LAND mode
#   "TOUCHDOWN"    — motors disarmed mid-track (defensive — at altitude
#                    this should not happen, but treat it as a clean
#                    exit if it does)
# ─────────────────────────────────────────────────────────────────────────────

def track_tag(controller, pump, state):
    print("[INFO] Phase: TRACK")
    state["phase"]     = "TRACK"
    state["leg_label"] = f"TRACK 0.0/{TRACK_DURATION_S:.0f}s"

    start         = time.time()
    last_tag_time = time.time()
    last_log      = 0.0
    centred_count = 0   # consecutive frames inside TRACK_CENTER_THRESHOLD_M

    # IMU recovery state — populated on the first frame after the marker
    # is lost; cleared the moment the marker re-appears.  All four
    # together describe "the airframe was moving this way relative to
    # itself when the marker disappeared, and the marker was last seen
    # over here in body frame" — that's enough to decide which way to
    # fly to recover the marker.
    drift_snapshot = None    # (body_vx, body_vy) m/s at loss
    last_tag_body  = None    # (body_x,  body_y)  m at last visible frame
    loss_start     = None    # wall-clock time when loss began
    last_recovery_log = 0.0

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
            # Re-acquisition (or first acquisition) — drop any recovery
            # state so the next loss starts with a fresh snapshot.
            if drift_snapshot is not None:
                print(f"[INFO] TRACK re-acquired tag after "
                      f"{time.time() - loss_start:.1f}s of IMU recovery")
                drift_snapshot = None
                loss_start = None

            last_tag_time = time.time()
            t = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])
            body_x, body_y, body_z = PrecisionLandingController.camera_to_body(
                cam_x, cam_y, cam_z,
            )
            last_tag_body = (body_x, body_y)

            # Frame freshness: pump() refreshes state["last_frame_time"]
            # only when a genuinely new in_rgb arrives.  If the detector
            # just re-ran on a cached frame, frame_age is the age of that
            # cached frame — we either dead-reckon-compensate inside
            # track_velocity_command or skip the PD entirely if too old.
            frame_time = state.get("last_frame_time")
            frame_age = (time.time() - frame_time) if frame_time else 0.0

            if frame_age > CAMERA_STALE_S:
                # Stale frame — do not feed dead-reckoned data through
                # the PD; the unmodeled component (wind drift since
                # capture) is no longer negligible at this age.  Break
                # the convergence streak too — a frame this old should
                # not be evidence of centring.
                centred_count = 0
                controller.send_velocity(0.0, 0.0, 0.0)
                now = time.time()
                if now - last_log > 1.0:
                    print(f"[INFO] TRACK stale frame (age={frame_age:.2f}s "
                          f"> {CAMERA_STALE_S:.2f}s) — holding zero velocity")
                    last_log = now
                time.sleep(0.05)
                continue

            filt_x, filt_y, vx, vy, vz = controller.track_velocity_command(
                body_x, body_y, body_z, frame_age=frame_age,
            )

            # Convergence: BOTH filtered axes inside the centre threshold
            # for TRACK_CENTER_HOLD_FRAMES consecutive ticks.  We use the
            # filtered values rather than raw body_* so a single noisy
            # frame doesn't break the streak.
            if (abs(filt_x) < TRACK_CENTER_THRESHOLD_M
                    and abs(filt_y) < TRACK_CENTER_THRESHOLD_M):
                centred_count += 1
            else:
                centred_count = 0
            if centred_count >= TRACK_CENTER_HOLD_FRAMES:
                controller.send_velocity(0.0, 0.0, 0.0)
                print(f"[INFO] TRACK centred (|x|<{TRACK_CENTER_THRESHOLD_M:.2f}, "
                      f"|y|<{TRACK_CENTER_THRESHOLD_M:.2f} for "
                      f"{TRACK_CENTER_HOLD_FRAMES} frames) — handing off "
                      "to PRECISION_LAND")
                return "READY"

            state["leg_label"] = (
                f"TRACK {elapsed:.1f}/{TRACK_DURATION_S:.0f}s "
                f"err=({body_x:+.2f},{body_y:+.2f})"
            )

            now = time.time()
            if now - last_log > 1.0:
                rel_alt = _relative_altitude_m(controller)
                alt_str = (f"{rel_alt:+.2f}/{TRACK_TARGET_ALT_M:.1f}m"
                           if rel_alt is not None else "n/a")
                print(f"[INFO] TRACK t={elapsed:.1f}/{TRACK_DURATION_S:.0f}s  "
                      f"body=({body_x:+.2f},{body_y:+.2f},{body_z:+.2f})  "
                      f"filt=({filt_x:+.2f},{filt_y:+.2f})  "
                      f"v=({vx:+.2f},{vy:+.2f},{vz:+.2f})  "
                      f"age={frame_age*1000:.0f}ms  "
                      f"alt={alt_str}  "
                      f"centred={centred_count}/{TRACK_CENTER_HOLD_FRAMES}")
                last_log = now
        else:
            # Tag dropped — break the convergence streak so a brief
            # detection on the next frame doesn't immediately satisfy it.
            centred_count = 0
            time_lost = time.time() - last_tag_time

            # First frame of loss: snapshot the body-frame drift NOW,
            # before the recovery counter command starts changing the
            # velocity.  Sampling later would just measure our own
            # counter command rather than the original drift.
            if drift_snapshot is None:
                loss_start = time.time()
                bvx, bvy, _ = controller.body_frame_velocity()
                if bvx is None or bvy is None:
                    # No yaw/velocity yet — degenerate snapshot.  The
                    # recovery method will fall back to last_tag_body
                    # if that's available, else hover.
                    bvx, bvy = 0.0, 0.0
                drift_snapshot = (bvx, bvy)
                print(f"[INFO] TRACK tag lost — IMU recovery snapshot "
                      f"body_v=({bvx:+.2f},{bvy:+.2f}) m/s, last_tag_body="
                      f"{last_tag_body}")

            recovery_elapsed = time.time() - loss_start
            if recovery_elapsed > RECOVERY_DURATION_S:
                print(f"[WARN] TRACK IMU recovery exhausted "
                      f"({recovery_elapsed:.1f}s without re-acquisition) — "
                      "committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            # Active recovery — send a body-frame counter-drift command.
            rec = controller.recover_velocity_command(
                drift_snapshot, last_tag_body=last_tag_body,
            )

            state["leg_label"] = (
                f"TRACK RECOVER {recovery_elapsed:.1f}/"
                f"{RECOVERY_DURATION_S:.1f}s "
                f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},{rec['vz']:+.2f}) "
                f"src={rec['source']}"
            )

            now = time.time()
            if now - last_recovery_log > 0.5:
                alt_str = (f"{rec['rel_alt']:+.2f}/{TRACK_TARGET_ALT_M:.1f}m"
                           if rec['rel_alt'] is not None else "n/a")
                print(f"[INFO] TRACK RECOVER t={recovery_elapsed:.1f}/"
                      f"{RECOVERY_DURATION_S:.1f}s  "
                      f"drift=({rec['drift_x']:+.2f},{rec['drift_y']:+.2f}) "
                      f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},"
                      f"{rec['vz']:+.2f})  "
                      f"alt={alt_str}  "
                      f"src={rec['source']}  "
                      f"oak={'yes' if rec['oak_confirm'] else 'no'}  "
                      f"scale={rec['scale']:.2f}")
                last_recovery_log = now

        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Precision-landing phase.  Returns one of:
#   "TOUCHDOWN"    — motors auto-disarmed; mission complete (also returned
#                    after a close-tag or EKF-altitude commit-to-LAND)
#   "COMMIT_LAND"  — IMU tag-loss recovery window expired without re-
#                    acquisition; caller commits to ArduCopter LAND mode
# ─────────────────────────────────────────────────────────────────────────────

def precision_land(controller, pump, state):
    """Velocity-PD descent in GUIDED mode, mirroring stationary_landing.py.

    The previous version of this function streamed LANDING_TARGET to the
    autopilot and switched to LAND mode immediately, relying on ArduCopter
    PrecLand to do the lateral correction.  On this airframe that lateral
    authority was too weak — a real flight log showed body-frame offsets
    persisting around ±1 m for the entire descent, and the drone slid past
    the tag laterally while LAND continued at its normal descent rate.

    The current version stays in GUIDED, runs a velocity-PD loop locally
    (descent_velocity_command), and only commits to LAND for the final
    ~0.6 m so ArduCopter handles ground-detection / auto-disarm.
    LANDING_TARGET is still published while the tag is visible — purely
    informational, so any PrecLand-aware setup downstream gets the data,
    but it is no longer the descent driver on this airframe.

    Tag-loss handling has two layers:

      1. CLOSE-TAG handoff: if the marker was last seen at body_z <
         CLOSE_TAG_BODY_Z_M (we're essentially on top of it), commit
         directly to LAND.  body_z is preferred over EKF altitude
         because flight logs showed EKF z drifting ~3 m mid-mission.

      2. IMU RECOVERY: for all other losses, capture the body-frame
         drift velocity at the moment of loss (Pixhawk EKF, cross-
         checked against the OAK-D S2 onboard IMU) and command the
         opposite velocity for up to RECOVERY_DURATION_S.  If the
         marker reappears we resume descent; if the window expires
         we commit to LAND (the user's spec; SEARCH is no longer
         triggered from PRECISION_LAND).

    Returns one of:
      * "TOUCHDOWN"   — motors auto-disarmed; mission complete
      * "COMMIT_LAND" — IMU recovery window expired; caller commits to LAND
    """
    print("[INFO] Phase: PRECISION_LAND")
    state["phase"] = "PRECISION_LAND"
    last_tag_time = time.time()
    last_body_z   = None
    last_tag_body = None      # (body_x, body_y) on last frame the tag was seen
    last_log      = 0.0

    # IMU recovery state — see the matching block in track_tag() for
    # the full rationale.  drift_snapshot is captured on the first
    # frame after loss and held constant throughout the recovery
    # window so a freshly-decelerated drone still keeps flying back.
    drift_snapshot = None
    loss_start     = None
    last_recovery_log = 0.0

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
            # Re-acquisition — clear recovery state so the next loss
            # snapshots fresh drift instead of reusing a stale one.
            if drift_snapshot is not None:
                print(f"[INFO] PRECISION_LAND re-acquired tag after "
                      f"{time.time() - loss_start:.1f}s of IMU recovery")
                drift_snapshot = None
                loss_start = None

            last_tag_time = time.time()
            t = tag.pose_t
            cam_x = float(t[0][0])
            cam_y = float(t[1][0])
            cam_z = float(t[2][0])

            body_x, body_y, body_z = PrecisionLandingController.camera_to_body(
                cam_x, cam_y, cam_z,
            )
            last_body_z   = body_z
            last_tag_body = (body_x, body_y)

            # Frame freshness: pump() refreshes last_frame_time only on
            # a brand-new in_rgb arrival.  Stale-frame skip and dead-
            # reckon-compensation rationale documented in track_tag().
            frame_time = state.get("last_frame_time")
            frame_age = (time.time() - frame_time) if frame_time else 0.0

            if frame_age > CAMERA_STALE_S:
                # Detection came from a cached frame older than
                # CAMERA_STALE_S — don't drive PD or commit-to-LAND on
                # stale data.  Hover and wait for a fresh detection.
                controller.send_velocity(0.0, 0.0, 0.0)
                now = time.time()
                if now - last_log > 1.0:
                    print(f"[INFO] DESCENT stale frame "
                          f"(age={frame_age:.2f}s > {CAMERA_STALE_S:.2f}s) "
                          "— holding zero velocity")
                    last_log = now
                time.sleep(0.02)
                continue

            # Drive the descent ourselves.  Returns the FILTERED body
            # offset so the commit-to-LAND gate below can require
            # cm-scale lateral alignment, not the raw (noisier) offset.
            filt_x, filt_y, vx, vy, vz = controller.descent_velocity_command(
                body_x, body_y, body_z, frame_age=frame_age,
            )

            # Final-approach handoff.  We commit to LAND only when either:
            #   (a) body_z < TOUCHDOWN_BODY_Z_M AND lateral alignment is
            #       tighter than TOUCHDOWN_XY_M (single-digit cm), OR
            #   (b) body_z < TOUCHDOWN_HARD_FLOOR_BZ_M — below that floor
            #       the altitude-scaled gains can't centre anyway, so
            #       prolonging PD would waste battery on an offset we
            #       cannot drive out.
            # If body_z is between the hard floor and TOUCHDOWN_BODY_Z_M
            # with a loose lateral, descent_velocity_command's XY_ERR_HOLD
            # branch is already throttling vz to 20 % so the drone keeps
            # nudging in laterally before sinking past the hard floor.
            lateral_err = max(abs(filt_x), abs(filt_y))
            if body_z < TOUCHDOWN_HARD_FLOOR_BZ_M:
                commit_to_land(
                    controller, pump,
                    f"body_z={body_z:.2f} m below hard floor "
                    f"TOUCHDOWN_HARD_FLOOR_BZ_M="
                    f"{TOUCHDOWN_HARD_FLOOR_BZ_M:.2f} m "
                    f"(lateral={lateral_err*100:.1f} cm)"
                )
                return "TOUCHDOWN"
            if body_z < TOUCHDOWN_BODY_Z_M:
                if lateral_err < TOUCHDOWN_XY_M:
                    commit_to_land(
                        controller, pump,
                        f"body_z={body_z:.2f} m below "
                        f"TOUCHDOWN_BODY_Z_M={TOUCHDOWN_BODY_Z_M:.2f} m "
                        f"and lateral={lateral_err*100:.1f} cm "
                        f"< TOUCHDOWN_XY_M={TOUCHDOWN_XY_M*100:.0f} cm"
                    )
                    return "TOUCHDOWN"
                # else: log once that we're deferring commit; the PD
                # continues to drive us laterally on the next tick.
                now = time.time()
                if now - last_log > 0.5:
                    print(f"[INFO] DESCENT deferring commit-to-LAND: "
                          f"body_z={body_z:.2f} m below "
                          f"TOUCHDOWN_BODY_Z_M={TOUCHDOWN_BODY_Z_M:.2f} m "
                          f"but lateral={lateral_err*100:.1f} cm "
                          f"> TOUCHDOWN_XY_M={TOUCHDOWN_XY_M*100:.0f} cm")
                    last_log = now

            # Publish LANDING_TARGET for any downstream PrecLand consumer.
            # On this airframe the descent is driven by descent_velocity_*
            # above; LANDING_TARGET here is purely informational.
            controller.send_landing_target(body_x, body_y, body_z)
            # descent_velocity_command clears last_cmd["lt_*"] so the HUD
            # falls through to the velocity branch — re-clear here in case
            # send_landing_target above re-populated them.
            controller.last_cmd["lt_x"] = None
            controller.last_cmd["lt_y"] = None
            controller.last_cmd["lt_z"] = None

            state["leg_label"] = (
                f"DESCENT bz={body_z:.2f}m err=({body_x:+.2f},{body_y:+.2f})"
            )

            now = time.time()
            if now - last_log > 0.5:
                if (controller.last_pos["z"] is not None and
                        controller.takeoff_z_origin is not None):
                    rel_alt = -(controller.last_pos["z"]
                                - controller.takeoff_z_origin)
                    rel_str = f"{rel_alt:+.2f} m rel"
                else:
                    rel_str = "n/a"
                print(f"[INFO] DESCENT body=({body_x:+.2f},"
                      f"{body_y:+.2f},{body_z:+.2f}) "
                      f"filt=({filt_x:+.2f},{filt_y:+.2f}) "
                      f"v=({vx:+.2f},{vy:+.2f},{vz:+.2f})  "
                      f"bz={body_z:.2f} m  age={frame_age*1000:.0f}ms  "
                      f"alt={rel_str}")
                last_log = now

        else:
            # Tag not visible this frame.
            elapsed = time.time() - last_tag_time

            # PRIMARY close-tag check: did we recently see the tag at
            # very low body_z?  Then we're nearly on top of it and it
            # is simply outside the FOV — commit to LAND, do NOT recover.
            # body_z is the camera-to-tag distance from AprilTag pose
            # estimation, which does not suffer the EKF drift.
            if (last_body_z is not None
                    and last_body_z < CLOSE_TAG_BODY_Z_M):
                commit_to_land(
                    controller, pump,
                    f"Tag too close to track (last bz={last_body_z:.2f} m)"
                )
                return "TOUCHDOWN"

            # First frame of loss: snapshot the body-frame drift NOW,
            # before the recovery counter command starts changing the
            # velocity.  Same rationale as in track_tag.
            if drift_snapshot is None:
                loss_start = time.time()
                bvx, bvy, _ = controller.body_frame_velocity()
                if bvx is None or bvy is None:
                    bvx, bvy = 0.0, 0.0
                drift_snapshot = (bvx, bvy)
                print(f"[INFO] PRECISION_LAND tag lost — IMU recovery "
                      f"snapshot body_v=({bvx:+.2f},{bvy:+.2f}) m/s, "
                      f"last_tag_body={last_tag_body}, "
                      f"last_bz={last_body_z}")

            recovery_elapsed = time.time() - loss_start
            if recovery_elapsed > RECOVERY_DURATION_S:
                # SECONDARY fallback (no body_z reading at all): use the
                # EKF-relative altitude.  Real flight logs showed EKF z
                # can drift several metres so this branch is fragile —
                # we only get here if the tag was never seen during
                # PRECISION_LAND in the first place (so no last_body_z).
                relative_alt = None
                if (controller.last_pos["z"] is not None and
                        controller.takeoff_z_origin is not None):
                    relative_alt = -(controller.last_pos["z"]
                                     - controller.takeoff_z_origin)
                if (last_body_z is None
                        and relative_alt is not None
                        and relative_alt < TAG_TOO_CLOSE_ALT_M):
                    commit_to_land(
                        controller, pump,
                        f"Tag never seen and EKF rel-alt "
                        f"{relative_alt:.2f} m below "
                        f"TAG_TOO_CLOSE_ALT_M={TAG_TOO_CLOSE_ALT_M:.2f} m"
                    )
                    return "TOUCHDOWN"
                print(f"[WARN] PRECISION_LAND IMU recovery exhausted "
                      f"({recovery_elapsed:.1f}s without re-acquisition) "
                      f"— total tag-loss {elapsed:.1f}s — committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            # Active recovery — body-frame counter-drift command.
            rec = controller.recover_velocity_command(
                drift_snapshot, last_tag_body=last_tag_body,
            )

            state["leg_label"] = (
                f"PL RECOVER {recovery_elapsed:.1f}/"
                f"{RECOVERY_DURATION_S:.1f}s "
                f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},{rec['vz']:+.2f}) "
                f"src={rec['source']}"
            )

            now = time.time()
            if now - last_recovery_log > 0.5:
                alt_str = (f"{rec['rel_alt']:+.2f}/{TRACK_TARGET_ALT_M:.1f}m"
                           if rec['rel_alt'] is not None else "n/a")
                print(f"[INFO] PRECISION_LAND RECOVER t="
                      f"{recovery_elapsed:.1f}/{RECOVERY_DURATION_S:.1f}s  "
                      f"drift=({rec['drift_x']:+.2f},{rec['drift_y']:+.2f}) "
                      f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},"
                      f"{rec['vz']:+.2f})  "
                      f"alt={alt_str}  "
                      f"src={rec['source']}  "
                      f"oak={'yes' if rec['oak_confirm'] else 'no'}  "
                      f"scale={rec['scale']:.2f}")
                last_recovery_log = now

        # ~50 Hz tick — comfortable on a Pi while leaving headroom for
        # AprilTag detection.
        time.sleep(0.02)


# ─────────────────────────────────────────────────────────────────────────────
# Search / re-locate — climb back, fly to last known tag spot, re-patrol.
# ─────────────────────────────────────────────────────────────────────────────

def _relative_altitude_m(controller):
    """Return current altitude above the takeoff anchor in metres, or None
    if either the LOCAL_POSITION_NED stream or the takeoff anchor has not
    been seeded yet.  Centralised here because every cap-related check
    needs the same anchored value, and a fall-through to raw -z would
    silently mis-cap on a stale-EKF-origin airframe.
    """
    cur_z = controller.last_pos.get("z")
    z0    = controller.takeoff_z_origin
    if cur_z is None or z0 is None:
        return None
    return -(cur_z - z0)


def safe_climb_to_altitude(controller, pump, state, target_relative_alt_m,
                           max_relative_alt_m=MAX_SEARCH_ALTITUDE_M):
    """Climb to ``target_relative_alt_m`` using body-frame velocity control
    with a hard cap at ``max_relative_alt_m`` (default
    MAX_SEARCH_ALTITUDE_M = TAKEOFF_ALTITUDE).

    Why not just goto_ned(z=takeoff_z_origin - target)?  A prior flight
    test of SEARCH showed the airframe overshooting the requested re-
    climb altitude by several metres before the autopilot's position
    controller settled.  goto_ned re-sends a position target every
    0.5 s and otherwise lets the autopilot do whatever it wants in
    between — there is no companion-side guard against the overshoot.
    A velocity-controlled climb monitored every 100 ms here can react
    to overshoot within ~one tick, and the hard cap immediately reverses
    to a descent command if the cap is breached.

    Exit conditions:
      * relative altitude is within SEARCH_TARGET_TOLERANCE_M of
        ``target_relative_alt_m`` → hover and return
      * SEARCH_CLIMB_TIMEOUT_S exceeded → hover and return (the lateral
        move can still continue at whatever altitude was achieved)
      * motors_armed drops to False (RC failsafe / ground hit) → return

    Detects the tag opportunistically every tick — if the marker is
    re-acquired before the climb completes, the caller can re-enter
    TRACK directly without finishing the climb.  Currently the caller
    does not consume this signal (returns plain None / behaviour
    matches the old flow); the detection is still useful for the HUD.
    """
    cap = max_relative_alt_m
    print(f"[INFO] SEARCH safe-climb target={target_relative_alt_m:.2f} m "
          f"(hard cap = {cap:.2f} m + "
          f"{SEARCH_CLIMB_OVERSHOOT_M:.2f} m overshoot)")

    start = time.time()
    last_log = 0.0

    while time.time() - start < SEARCH_CLIMB_TIMEOUT_S:
        controller._drain_messages()
        if not controller.master.motors_armed():
            print("[INFO] Motors disarmed mid-SEARCH-climb — bailing.")
            return

        rel_alt = _relative_altitude_m(controller)
        if rel_alt is None:
            # No altitude reading yet — hover and wait one tick.
            controller.send_velocity(0.0, 0.0, 0.0)
            state["leg_label"] = "CLIMB  (waiting for altitude)"
            pump(detect=True)
            time.sleep(0.1)
            continue

        if rel_alt > cap + SEARCH_CLIMB_OVERSHOOT_M:
            # HARD CAP BREACHED — actively descend to bring us back below
            # the cap.  This is the safety net the prior flight test was
            # missing.
            controller.send_velocity(0.0, 0.0, SEARCH_DESCEND_VZ)
            state["leg_label"] = (
                f"CLIMB CAP {rel_alt:+.2f}m > {cap:.2f}+"
                f"{SEARCH_CLIMB_OVERSHOOT_M:.1f}  → DESCEND"
            )
            print(f"[WARN] SEARCH altitude cap breached "
                  f"({rel_alt:+.2f} m > {cap:.2f}+"
                  f"{SEARCH_CLIMB_OVERSHOOT_M:.1f}) — descending")
        elif rel_alt >= target_relative_alt_m - SEARCH_TARGET_TOLERANCE_M:
            # Target reached (or exceeded but within overshoot margin).
            controller.send_velocity(0.0, 0.0, 0.0)
            print(f"[INFO] SEARCH target altitude reached "
                  f"({rel_alt:+.2f} m vs target "
                  f"{target_relative_alt_m:.2f} m)")
            return
        else:
            # Continue climbing at the configured rate.
            controller.send_velocity(0.0, 0.0, SEARCH_CLIMB_VZ)
            state["leg_label"] = (
                f"CLIMB {rel_alt:+.2f}/"
                f"{target_relative_alt_m:.1f}m  cap={cap:.1f}"
            )

        now = time.time()
        if now - last_log > 1.0:
            print(f"[INFO] SEARCH climb alt={rel_alt:+.2f} m  "
                  f"target={target_relative_alt_m:.2f} m  "
                  f"cap={cap:.2f}+{SEARCH_CLIMB_OVERSHOOT_M:.1f} m")
            last_log = now

        pump(detect=True)
        time.sleep(0.1)

    print(f"[WARN] SEARCH climb timeout ({SEARCH_CLIMB_TIMEOUT_S:.0f} s) — "
          "proceeding at whatever altitude was achieved")
    controller.send_velocity(0.0, 0.0, 0.0)


def search_and_relocate(controller, pump, state, last_known_xy):
    """Switch to GUIDED, climb back to TAKEOFF_ALTITUDE (with a hard
    altitude cap), fly to the last known marker (x, y), and re-run the
    box patrol centered there.

    Climb portion uses safe_climb_to_altitude() — velocity-controlled
    with a hard cap at MAX_SEARCH_ALTITUDE_M so the airframe cannot
    overshoot the way it did in a prior flight test.

    Lateral move uses goto_ned() at the achieved altitude (not the
    requested target), and the pump callback we hand goto_ned actively
    watches relative altitude every tick — if the autopilot drifts the
    drone above the cap during the lateral flight, the watchdog
    injects a corrective vz>0 body-frame velocity command (which
    overrides the goto position target until goto_ned's next 0.5 s
    re-send).  This is a soft safety net for the lateral phase; the
    primary defence is the climb-phase cap above.

    Returns the new (last_known_x, last_known_y) if the tag is re-acquired
    during the re-patrol, or ``None`` if the box completed without a hit.
    """
    print("[INFO] Phase: SEARCH")
    state["phase"]     = "SEARCH"
    state["leg_label"] = "CLIMB"

    controller.change_flight_mode("GUIDED")

    last_x, last_y = last_known_xy

    # ── Climb phase: velocity-controlled with hard altitude cap ──────────
    # Target = MAX_SEARCH_ALTITUDE_M (=TAKEOFF_ALTITUDE per user spec).
    safe_climb_to_altitude(
        controller, pump, state,
        target_relative_alt_m=MAX_SEARCH_ALTITUDE_M,
        max_relative_alt_m=MAX_SEARCH_ALTITUDE_M,
    )

    # ── Lateral move: goto_ned at the ACHIEVED altitude ─────────────────
    # Compute z target from the current LOCAL_POSITION_NED.z so the
    # autopilot is told "hold this altitude, just move x/y" rather than
    # being asked to climb further.  Fall back chain mirrors the old
    # search_and_relocate so a missing takeoff anchor still produces a
    # usable (if approximate) z target.
    cur_z = controller.last_pos.get("z")
    if cur_z is not None:
        target_z = cur_z
    elif controller.takeoff_z_origin is not None:
        target_z = controller.takeoff_z_origin - MAX_SEARCH_ALTITUDE_M
    else:
        print("[WARN] No takeoff anchor and no LOCAL_POSITION_NED — "
              "search_and_relocate falling back to "
              f"target_z=-{MAX_SEARCH_ALTITUDE_M:.2f} (may be wrong by "
              "the EKF-origin offset on stale-origin airframes)")
        target_z = -MAX_SEARCH_ALTITUDE_M

    def _capped_pump():
        """pump() wrapper that also enforces the altitude cap during
        the lateral goto.  If the autopilot drifts above the cap +
        overshoot margin, send a body-frame descent command — this
        will be overridden by goto_ned's next 0.5 s position re-send,
        but that's fine: it limits sustained over-cap flight to
        roughly one goto re-send interval.
        """
        rel_alt = _relative_altitude_m(controller)
        if (rel_alt is not None
                and rel_alt > MAX_SEARCH_ALTITUDE_M + SEARCH_CLIMB_OVERSHOOT_M):
            controller.send_velocity(0.0, 0.0, SEARCH_DESCEND_VZ)
            print(f"[WARN] SEARCH lateral-move altitude cap breached "
                  f"({rel_alt:+.2f} m) — injecting descent")
        return pump(detect=True)

    controller.goto_ned(
        last_x, last_y, target_z,
        tolerance=GOTO_TOLERANCE, timeout=GOTO_TIMEOUT,
        pump_fn=_capped_pump,
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

        # OAK-D S2 onboard BNO086 IMU — added for the new tag-loss
        # recovery (see RECOVERY_* config above).  We enable only the
        # accelerometer; gyro is not needed for the cross-check at this
        # time but is cheap to add later if attitude rate-of-change
        # becomes useful.  Sample rate matches RECOVERY_OAK_IMU_HZ so
        # the EMA filter in make_pump can settle within a few frames.
        oak_imu = pipeline.create(dai.node.IMU)
        oak_imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, RECOVERY_OAK_IMU_HZ)
        oak_imu.setBatchReportThreshold(1)
        oak_imu.setMaxBatchReports(10)
        q_oak_imu = oak_imu.out.createOutputQueue(maxSize=20, blocking=False)

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
        pump = make_pump(q_rgb, q_oak_imu, detector, controller, state)

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
            # ── TRACK → PRECISION_LAND ──────────────────────────────────────
            # With the IMU tag-loss recovery in place, neither phase returns
            # "TAG_LOST" anymore — the recovery either re-acquires the
            # marker (and the phase continues) or its 3-second window
            # expires and we get "COMMIT_LAND".  In both COMMIT_LAND cases
            # the user's spec says: skip search_and_relocate (the
            # box-re-patrol path) entirely and commit straight to LAND.
            track_result = track_tag(controller, pump, state)
            if track_result == "TOUCHDOWN":
                state["phase"] = "TOUCHDOWN"
            elif track_result == "COMMIT_LAND":
                state["phase"] = "TOUCHDOWN"
                commit_to_land(
                    controller, pump,
                    "TRACK IMU recovery exhausted — final commit",
                )
            else:
                # track_result == "READY" → descent phase.
                result = precision_land(controller, pump, state)
                if result == "TOUCHDOWN":
                    state["phase"] = "TOUCHDOWN"
                elif result == "COMMIT_LAND":
                    state["phase"] = "TOUCHDOWN"
                    commit_to_land(
                        controller, pump,
                        "PRECISION_LAND IMU recovery exhausted — final commit",
                    )

        # Final pump so the very last HUD frame is visible briefly before
        # window teardown.
        for _ in range(20):
            pump()
            time.sleep(0.05)

cv2.destroyAllWindows()
