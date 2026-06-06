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
   INIT → STABILIZE → ACQUIRE ─tag─▶ TRACK ─centred/timeout─▶ PRECISION_LAND
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
  frames; TRACK_DURATION_S is just an upper bound (currently 25 s).  If
  the upper bound is reached without the filtered lateral error sitting
  inside TRACK_HANDOFF_LATERAL_M we treat TRACK as having failed to
  centre and commit to LAND in place rather than hand an off-centre
  hover to PRECISION_LAND (which would just descend obliquely).

   Within PRECISION_LAND:
     * Mode is GUIDED while body_z > TOUCHDOWN_BODY_Z_M; control source
       is ``descent_velocity_command`` (companion-side velocity-PD).
     * Mode flips to LAND once body_z < TOUCHDOWN_BODY_Z_M; control
       source is ArduCopter LAND for ground-detection + auto-disarm.

   IMU RECOVERY (see RECOVERY_* config) runs whenever the AprilTag
   leaves the camera frame during TRACK or PRECISION_LAND.  At the
   moment of loss it captures the Pixhawk EKF NED position as
   ``loss_anchor`` and runs a closed-loop position-PD that flies the
   airframe back to that anchor while a separate altitude P-loop
   drives it back to TAKEOFF_ALTITUDE — successive losses converge on
   the same set altitude rather than ratcheting upward.  The OAK-D
   S2 BNO086 accelerometer is used as a sanity cross-check on the
   EKF velocity reading (gain attenuation only — the loop itself
   closes around EKF position).  The window is RECOVERY_DURATION_S
   (3 s); if the marker reappears we resume the parent phase, and if
   the window expires (or we've returned to within
   RECOVERY_ANCHOR_RADIUS_M of the loss anchor without re-acquiring)
   we commit straight to ArduCopter LAND.  The old
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
import cv2.aruco as aruco
import numpy as np
import depthai as dai
from pymavlink import mavutil

# pyserial — used both by pymavlink (Pixhawk link) and by the LoRa UGV link
# below.  Guarded so a missing module degrades to "no UGV link" rather than
# aborting the whole flight script.
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ─────────────────────────────────────────────────────────────────────────────
# Flight Config
# ─────────────────────────────────────────────────────────────────────────────

CONNECTION_STRING = "/dev/serial0"
BAUDRATE          = 57600
TAKEOFF_ALTITUDE  = 3.0           # meters
TAKEOFF_ALT_TOLERANCE_M = 0.20  # m — authoritative-altitude band for declaring
                                # the takeoff target reached (depth-preferred,
                                # see _altitude_reading / _relative_altitude_m).
TAKEOFF_TIMEOUT_S       = 30.0  # s — overall climb-monitor budget before we
                                # warn and proceed (was an inline literal).

# ─────────────────────────────────────────────────────────────────────────────
# Stabilization Config (NEW — not present in stationary_landing.py /
# patrol_landing.py).  Before we start the patrol we verify the autopilot is
# actually holding altitude and not drifting laterally.  The values that come
# OUT of stabilization are used as the (x_home, y_home) anchor for the patrol.
# ─────────────────────────────────────────────────────────────────────────────

STABILIZE_ALT_TOLERANCE   = 0.3   # m       — ±0.3 m around TAKEOFF_ALTITUDE
STABILIZE_DRIFT_TOLERANCE = 0.5   # m       — horizontal drift from origin
                                  # (informational only — no longer gates)
STABILIZE_VEL_TOLERANCE   = 0.3   # m/s     — max horizontal velocity
                                  # (informational only — no longer gates)
STABILIZE_HOLD_SECONDS    = 0.5   # how long the altitude check must hold true
                                  # before proceeding — a quick sanity confirm,
                                  # not a long multi-criteria settle.
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

# Acquire-tag phase: replaces the box patrol per user spec — after STABILIZE
# the drone hovers in place and waits for the AprilTag to appear in the
# camera frame, then hands directly off to TRACK.  Assumes the tag is at
# (or very near) the takeoff point.  If the timeout expires without a
# detection we fall back to plain LAND at the current spot (same fallback
# the patrol path used when it returned None).
ACQUIRE_TIMEOUT_S = 30.0
# Quick tag-visibility gate before the combined track-and-descend phase.
# After the fast altitude confirm we do a SHORT check that the AprilTag is
# actually in view so the drone never descends blind; if it is not seen
# within this window we fall back to plain LAND at the current position
# (same fallback as an ACQUIRE timeout).
TAG_GATE_TIMEOUT_S = 5.0
# Hover-and-scan window at the start of PRECISION_LAND when the marker has
# never been seen yet.  Without this the IMU-recovery timer starts on the
# first missed frame (last_tag_time is initialised at entry) and force-lands
# after ~RECOVERY_DURATION_S even though the detector was still warming up.
PRECISION_LAND_ACQUIRE_S = 20.0

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

TAG_LOSS_TIMEOUT       = 6.0      # s — if tag stays missing this long during
                                  # PRECISION_LAND we bail out, climb back,
                                  # and re-fly the box at the last known spot.

MAX_RESEARCH_ATTEMPTS  = 3        # exhaust then fall back to plain LAND mode

GOTO_TOLERANCE         = 0.5      # m — goto_ned() accept radius
GOTO_TIMEOUT           = 20.0     # s — goto_ned() blocking upper bound

# NOTE: the former CLOSE_TAG_BODY_Z_M / TAG_TOO_CLOSE_ALT_M "tag too close →
# commit to LAND" handoffs were REMOVED.  They existed because the single
# large AprilTag overflowed the FOV at close range, so a tag-loss below ~1 m
# was assumed to be "we're on top of it" rather than drift.  The nested board's
# INNER tag (#12) stays resolvable through the close-range approach, so a loss
# now genuinely means drift (handled by IMU recovery) or true touchdown
# (handled by the TOUCHDOWN_BODY_Z_M / TOUCHDOWN_HARD_FLOOR_BZ_M commit while
# the tag is still visible).

TAG_COAST_S            = 0.8      # s — brief-dropout grace window during
                                  # PRECISION_LAND.  A tilt/shake/motion-blur
                                  # dropout is re-acquired in well under a
                                  # second; for the first TAG_COAST_S of a
                                  # loss we COAST on the last good centring
                                  # command (vertical paused) instead of
                                  # dropping into the heavier IMU recovery.
                                  # Recovery (and its loss/commit accounting)
                                  # only starts once a loss outlasts this
                                  # window, so intermittent quickly-re-acquired
                                  # dropouts never stack toward a forced LAND.

# ── Descent (manual velocity-PD, GUIDED mode) ──────────────────────────────
# When precision-landing on the tag we run a PD controller ourselves rather
# than rely on ArduCopter PrecLand's lateral correction (which on this
# airframe was too weak to keep the camera centred — confirmed in flight
# logs where body-frame offsets persisted around ±1 m through the entire
# descent).  Mirrors the design in
# UAV/PixhawkController/stationary_landing_controller.py.
DESCENT_Kp_XY            = 0.55  # was 0.35 — the descent kept drifting off the
                                 # tag (body-x grew 0.07→2.0 m) because the
                                 # unsaturated PD output at ~1 m error (0.35
                                 # m/s) was below the lateral drift rate, so
                                 # the error grew instead of shrinking.  0.55
                                 # commands ~0.55 m/s at 1 m error — closer to
                                 # the 0.70 m/s cap — so it actually outpaces
                                 # the drift and re-centres promptly.
DESCENT_Kd_XY            = 0.30  # was 0.25 — extra damping to absorb camera
                                 # pipeline latency (drone keeps moving for a
                                 # frame before the next detection updates).
DESCENT_MAX_V_XY         = 0.70  # m/s.  Raised from 0.4 m/s after a real
                                 # outdoor flight test of TRACK / DESCENT
                                 # showed sustained wind drift saturating
                                 # the previous 0.35–0.40 m/s caps while
                                 # body-frame offset GROWED rather than
                                 # shrank (controller permanently fighting
                                 # wind it could not outpace).  At
                                 # DESCENT_Kp_XY = 0.35 the unsaturated PD
                                 # output at 1 m error is ~0.35 m/s, so
                                 # 0.70 m/s only bites at err > ~2 m — a
                                 # single noisy detection at small error
                                 # cannot snap to the cap.  Retune in
                                 # flight if a more aggressive descent
                                 # is required.
DESCENT_TARGET_BZ        = 0.3   # m, desired height above tag during PD
DESCENT_Kp_Z             = 0.3
DESCENT_MIN_VZ           = 0.10  # m/s minimum descent rate when centred
DESCENT_MAX_VZ           = 0.60  # m/s vertical clamp.  Was 0.40 — raised so
                                 # the descent is brisk ONCE the tag is inside
                                 # the centring cone (see DESCENT_XY_ERR_HOLD*);
                                 # it only bites when we are actually centred.
DESCENT_XY_ERR_HOLD      = 0.10  # m — near-ground floor of the descent gate.
                                 # When the lateral error exceeds the gate the
                                 # vertical command is now PAUSED (vz=0), not
                                 # merely throttled — recentre first so the tag
                                 # cannot drift to the FOV edge and drop out
                                 # while we keep sinking.
DESCENT_XY_ERR_HOLD_FRAC = 0.15  # — altitude-proportional part of the descent
                                 # gate: descend only while the lateral error
                                 # is within max(DESCENT_XY_ERR_HOLD,
                                 # FRAC*body_z) — i.e. inside a ~15 % cone of
                                 # the current height.  Generous at altitude
                                 # (0.66 m at 4.4 m) so the descent stays brisk,
                                 # tightening to the cm floor near touchdown.
DESCENT_DEADBAND_XY      = 0.005 # m — ignore offsets below ~5 mm.
                                 # Reduced from 0.03 m (same rationale as
                                 # TRACK_DEADBAND_XY) so the descent PD
                                 # keeps driving the tag toward pixel-center
                                 # down to sub-cm accuracy.
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
CAMERA_STALE_S           = 0.90  # s — older than this, treat the detection
                                 # as unusable for PD.  Raised from 0.50 s
                                 # after a tilt/shake flight test: when the
                                 # tag blurs, the detector's preprocessing
                                 # cascade runs deep before it finds (or
                                 # fails to find) the tag, so frame_age — which
                                 # tracks detection latency, not just camera
                                 # cadence — climbed to 0.69–0.76 s.  At the
                                 # old 0.50 s threshold those genuinely-valid
                                 # late detections were discarded as "stale",
                                 # so the controller stopped centring/descending
                                 # exactly when the shake demanded it and drifted
                                 # into recovery.  0.90 s lets a slow-but-real
                                 # detection drive the PD (dead-reckon-bounded
                                 # by DESCENT_DEADRECKON_MAX_S below), while a
                                 # truly dropped frame still trips the branch.
DESCENT_DEADRECKON_MAX_S = 0.35  # s — cap on the frame_age used for the
                                 # descent dead-reckon lead term.  CAMERA_STALE_S
                                 # now admits detections up to 0.90 s old, but
                                 # subtracting last_v * 0.90 could over-correct
                                 # (or sign-flip) the measured offset on a frame
                                 # where the airframe did NOT actually travel
                                 # the full commanded distance; clamping the
                                 # lead to 0.35 s bounds that correction.

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

TRACK_DURATION_S       = 25.0     # s — upper bound on TRACK; converging
                                  # below TRACK_CENTER_THRESHOLD_M for
                                  # TRACK_CENTER_HOLD_FRAMES exits earlier.
                                  # Raised from 12 s after a real flight
                                  # test where the velocity cap was below
                                  # the wind drift, leaving the controller
                                  # permanently saturated and unable to
                                  # close the gap inside 12 s.  Now that
                                  # TRACK_MAX_V_XY is well above the
                                  # observed wind drift, 25 s is enough
                                  # time to drag a multi-metre opening
                                  # offset down to the 5 cm convergence
                                  # threshold even with brief stale-frame
                                  # windows along the way.  If the upper
                                  # bound is reached without the filtered
                                  # lateral error being inside
                                  # TRACK_HANDOFF_LATERAL_M we abort to
                                  # LAND in place instead of handing an
                                  # off-centre hover to PRECISION_LAND.
TRACK_LOSS_TIMEOUT_S   = 6.0      # s — bail to SEARCH after this much loss
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
TRACK_MAX_V_XY         = 0.80     # m/s — per-axis horizontal cap.  Raised
                                  # from 0.35 m/s after a real outdoor
                                  # flight test where the controller was
                                  # commanding the cap on both axes
                                  # (v=(-0.35,-0.35)) while body offset
                                  # GROWED from (-0.54,+0.40) to
                                  # (-1.48,-1.47) over ~9 s — i.e. wind
                                  # drift exceeded the cap and the
                                  # controller had zero authority to
                                  # close the gap.  0.80 m/s gives ~0.45
                                  # m/s of headroom over the observed
                                  # drift, while still well above the
                                  # ~0.35 m/s the PD naturally produces
                                  # at 1 m error (so a single noisy
                                  # detection at typical errors does not
                                  # snap to the cap).  Tune up further if
                                  # the airframe still cannot outrun the
                                  # wind on a given day.
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
TRACK_DEADBAND_XY      = 0.005    # m — ignore offsets below ~5 mm.
                                  # Reduced from 0.03 m so the controller
                                  # keeps driving toward zero even at
                                  # cm-scale errors; AprilTag pose noise
                                  # at cruise altitude is sub-mm after
                                  # EMA filtering so this does not chase
                                  # noise.  Tighter deadband is the key
                                  # fix for the "not centering" issue:
                                  # with a 3 cm deadband the drone could
                                  # stall 2–3 cm off-center and never
                                  # reach pixel error ≈ 0.
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
TRACK_HANDOFF_LATERAL_M  = 0.15   # m — soft handoff acceptance.  If TRACK
                                  # hits TRACK_DURATION_S WITHOUT the
                                  # CENTER_HOLD_FRAMES streak but the most
                                  # recent filtered lateral error is
                                  # inside this radius, hand off anyway —
                                  # PRECISION_LAND's PD can continue to
                                  # close the residual gap during descent.
                                  # If the most recent error is OUTSIDE
                                  # this radius we treat TRACK as having
                                  # genuinely failed to centre (real
                                  # cause: wind drift exceeding the
                                  # velocity cap, or the tag stayed out
                                  # of FOV for most of the window) and
                                  # commit to LAND in place — descending
                                  # at >0.15 m off-centre would only end
                                  # up landing on the tag's neighbourhood,
                                  # not the tag itself, and risks the tag
                                  # leaving the FOV before touchdown.

# ─────────────────────────────────────────────────────────────────────────────
# IMU Tag-Loss Recovery Config
# ─────────────────────────────────────────────────────────────────────────────
# When the AprilTag falls out of the camera frame mid-flight (most likely
# cause: a wind gust pushing the airframe laterally), we no longer just
# hover during the loss window — that lets the drift continue and almost
# always ends in SEARCH or a bad commit-to-LAND off-target.
#
# The recovery is now a CLOSED-LOOP POSITION RECOVERY anchored at the
# Pixhawk-EKF NED position the airframe was at the instant the marker
# was lost (``loss_anchor_ned``).  Each tick we read the current EKF
# NED position and command a velocity that drives the airframe back
# toward that anchor:
#
#     v_world_xy = RECOVERY_POS_KP * (anchor_xy - current_xy)
#                + RECOVERY_POS_KD * (-current_world_velocity_xy)
#
# The +Kd term explicitly damps any wind-induced motion that the EKF
# is still measuring — the controller is closing the loop on actual
# position, not on a one-shot drift snapshot, so a sustained gust no
# longer beats us into a stagnant hover.  The world-frame command is
# then rotated into body frame (using current ATTITUDE.yaw) before
# being shipped through ``send_velocity`` (MAV_FRAME_BODY_NED).
#
# Vertical control during recovery is a separate closed loop on the
# anchored relative altitude with target = TAKEOFF_ALTITUDE — using the
# loss-anchor's z would carry forward the descent altitude that
# PRECISION_LAND was at when it lost the tag, which is the opposite of
# what we want.  Successive losses therefore converge on the SAME set
# altitude rather than ratcheting higher each time.
#
# Cross-check sources (used for confidence scaling, not as the loop
# itself):
#   PRIMARY  — Pixhawk LOCAL_POSITION_NED (x, y, vx, vy) — the EKF's
#              fused estimate.  This is the only source the loop
#              actually closes around.
#   BACKUP   — OAK-D S2 onboard BNO086 accelerometer (ACCELEROMETER_RAW),
#              mapped from camera frame to body frame with the existing
#              camera_to_body() transform.  Used purely as a sanity
#              cross-check: if the EKF reports meaningful body-frame
#              velocity but the OAK IMU sees no matching lateral
#              acceleration we attenuate the lateral command to
#              RECOVERY_OAK_DISAGREE_SCALE (a soft "trust EKF less"
#              when the two IMU stacks disagree).  ``drift_snapshot``
#              taken at the moment of loss is still passed through for
#              this check and for HUD/log purposes.
#
# If the marker re-appears before RECOVERY_DURATION_S we exit recovery
# and resume the parent phase (TRACK or PRECISION_LAND).  If the window
# expires without re-acquisition we commit directly to ArduCopter LAND
# (NOT search_and_relocate) per the user spec.  Optionally, if we
# return to within RECOVERY_ANCHOR_RADIUS_M of the loss anchor and
# still have not re-acquired, the marker is deemed truly gone and we
# commit to LAND early rather than running out the timer.

RECOVERY_DURATION_S        = 6.0   # s — must match TAG_LOSS_TIMEOUT and
                                   # TRACK_LOSS_TIMEOUT_S.  Extended from 3.0
                                   # so the ascend-to-re-acquire climb has
                                   # enough time to widen the FOV and re-find
                                   # the tag after a low loss before committing
                                   # to LAND.
RECOVERY_POS_KP            = 0.8   # P gain on the world-frame position
                                   # error (anchor_xy - current_xy).  Tune
                                   # up for snappier recovery, down if the
                                   # airframe overshoots the anchor.
RECOVERY_POS_KD            = 0.6   # D gain on the world-frame velocity.
                                   # Acts as direct damping on whatever
                                   # the EKF is currently measuring —
                                   # this is the term that fights an
                                   # ongoing wind gust.  Tune up if the
                                   # response oscillates around the
                                   # anchor; down if it feels sluggish.
RECOVERY_MAX_V_XY          = 0.4   # m/s — per-axis clamp on the body-frame
                                   # counter command after rotation from
                                   # world frame.
RECOVERY_MIN_V_XY          = 0.10  # m/s — minimum magnitude per axis once
                                   # the position error on that axis is
                                   # above RECOVERY_POS_DEADBAND_M; ensures
                                   # the airframe visibly moves instead of
                                   # collapsing under the cap on a small
                                   # but real error.
RECOVERY_POS_DEADBAND_M    = 0.05  # m — world-frame position error per
                                   # axis below this is treated as zero
                                   # (no lateral command on that axis).
RECOVERY_ANCHOR_RADIUS_M   = 0.30  # m — if the airframe returns to within
                                   # this horizontal radius of the loss
                                   # anchor and still has not re-acquired
                                   # the tag, commit to LAND early rather
                                   # than running out RECOVERY_DURATION_S.
RECOVERY_ALT_KP            = 0.6   # gain on (TAKEOFF_ALTITUDE - rel_alt)
                                   # for the recovery vertical loop.
                                   # Independent from TRACK_Kp_Z so the
                                   # recovery climb-back can be tuned
                                   # without changing TRACK behaviour.
RECOVERY_MAX_VZ            = 0.8   # m/s — per-axis vertical clamp during
                                   # recovery.  Well above TRACK_MAX_VZ so a
                                   # recovery that starts well below
                                   # TAKEOFF_ALTITUDE (e.g. a low mid-descent
                                   # loss) climbs back quickly to widen the
                                   # FOV and re-acquire the tag.  The hard
                                   # TAKEOFF_ALTITUDE cap in
                                   # recover_velocity_command still prevents
                                   # any climb past the set altitude.
RECOVERY_ALT_DEADBAND_M    = 0.10  # m — relative-altitude error below this
                                   # is treated as on-target (vz = 0).
RECOVERY_DRIFT_DEADBAND    = 0.05  # m/s — body-frame drift snapshot below
                                   # this on both axes is treated as
                                   # "EKF saw no motion at loss"; used by
                                   # the OAK confidence check.
RECOVERY_OAK_ACCEL_MIN     = 0.30  # m/s² — minimum OAK-D lateral-accel
                                   # magnitude (XY in body frame, gravity
                                   # is on body-Z for a downward camera so
                                   # XY is gravity-free to first order) to
                                   # count as "the camera IMU sees motion"
RECOVERY_OAK_DISAGREE_SCALE = 0.6  # gain scale when the EKF reports
                                   # body-frame motion but the OAK IMU
                                   # does NOT confirm it; we still close
                                   # the position loop but at reduced
                                   # lateral authority.
RECOVERY_OAK_EMA_ALPHA     = 0.7   # EMA on OAK accel samples in the pump
                                   # (heavy filter — BNO086 raw is noisy
                                   # at the 100 Hz pipeline rate)
RECOVERY_OAK_IMU_HZ        = 100   # OAK IMU sample rate for both the
                                   # accelerometer and gyroscope streams

# ─────────────────────────────────────────────────────────────────────────────
# GPS/NED Wind-Correction Config (Issue 3)
# When no AprilTag correction is driving the drone (ACQUIRE phase, or
# whenever the drone should hold a fixed world-position), compare the
# Pixhawk EKF NED position against the saved anchor and issue a corrective
# body-frame velocity to fight any wind-induced drift.  This is a pure
# PHYSICAL position check — no time-based or motor-estimate approach.
#
# Logic:
#   err_world  = (anchor_ned_xy - current_ned_xy)
#   if |err_world| > WIND_CORRECTION_THRESHOLD_M:
#       v_world_xy = WIND_CORRECTION_KP * err_world   (clamped)
#       v_body_xy  = R(yaw) · v_world_xy              (NED → body)
#       send_velocity(v_body_xy, vz_alt_hold)
#
# The anchor is updated whenever the drone INTENTIONALLY moves (e.g. at
# the end of acquire_tag() when the tag-tracking loop takes over).
# ─────────────────────────────────────────────────────────────────────────────

WIND_CORRECTION_KP           = 0.5   # P gain on world-frame position error
                                     # (m/s per m of drift).
WIND_CORRECTION_DEADBAND_M   = 0.05  # m per axis — ignore tiny GPS noise
WIND_CORRECTION_THRESHOLD_M  = 0.20  # m — minimum drift magnitude before
                                     # the correction is applied at all.
                                     # Below this the drone is considered
                                     # "on target" and no lateral command
                                     # is issued.
WIND_CORRECTION_MAX_V        = 0.30  # m/s — per-axis body-frame clamp on
                                     # the wind-correction command.  Kept
                                     # below PATROL_SPEED so the correction
                                     # never out-races a patrol leg command.

# ─────────────────────────────────────────────────────────────────────────────
# DepthAI OAK-D S2 Stereo-Depth Altitude Config (Issue 4)
# Replace Pixhawk IMU altitude with the stereo depth at the centre pixel of
# the OAK-D S2 depth frame.  The depth at (cx, cy) approximates the slant
# range from camera to the surface directly below — which equals the drone's
# AGL altitude when the drone is level.
#
# CRITICAL SAFETY CONSTRAINT: when pitch or roll exceeds
# DEPTH_ALT_TILT_THRESHOLD_DEG the camera is no longer pointing straight
# down and the centre-pixel depth is NOT the AGL altitude.  We silently
# fall back to the Pixhawk EKF altitude in that case.  Pitch/roll are read
# from the MAVLink ATTITUDE message cached in controller.last_att.
# ─────────────────────────────────────────────────────────────────────────────

DEPTH_ALT_TILT_THRESHOLD_DEG = 5.0   # degrees — max |pitch| or |roll| to
                                     # allow depth-based altitude.  Above
                                     # this fall back to Pixhawk EKF.
DEPTH_ALT_VALID_MIN_M        = 0.20  # m — depth values below this are sensor
                                     # noise / too-close; discard.
DEPTH_ALT_VALID_MAX_M        = 20.0  # m — depth values above this are
                                     # spurious; discard.
DEPTH_ALT_STALE_S            = 0.5   # s — max age of the last valid depth
                                     # reading; older → fall back to EKF.
DEPTH_ALT_EMA_ALPHA          = 0.70  # EMA weight on previous depth reading
                                     # to smooth frame-to-frame noise from
                                     # the stereo pipeline.
DEPTH_STEREO_RES             = (640, 400)  # resolution for left/right cameras
                                           # feeding the StereoDepth node.

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

# ── Nested 36h11 marker (two layers) ───────────────────────────────────────
# The landing target is a NESTED AprilTag board: a large OUTER tag with a
# small INNER tag printed inside its white centre (see
# UAV/Detectors/nested_aruco_detector.py and the Nested_AprilTags generator).
#   * OUTER id 77  — acquired from far away / at altitude.
#   * INNER id 12  — fixed by the board generator; stays resolvable at very
#                    close range when the OUTER tag overflows the FOV.
# Detection ALWAYS prioritises the inner tag: whenever #12 is visible it is
# used for tracking / landing, falling back to #77 only when #12 is not yet
# resolvable.  Because the inner tag survives the close-range approach there
# is no longer a "tag left the FOV because we got too close" case to special-
# case — the old CLOSE_TAG_BODY_Z_M / TAG_TOO_CLOSE_ALT_M handoffs are gone.
OUTER_TAG_ID   = 77            # outer tag id of the printed board
INNER_TAG_ID   = 12            # inner tag id — fixed by generate_board_nest.py
OUTER_TAG_SIZE = 0.20          # m — outer black-square side length
INNER_TAG_SIZE = OUTER_TAG_SIZE / 4.0   # m — inner black-square side length
ARUCO_DICT     = aruco.DICT_APRILTAG_36h11
NESTED_MAX_MISSING_FRAMES = 4  # temporal cache: bridge brief dropouts

# Representative tag size for the (informational) LANDING_TARGET payload —
# the outer tag is the larger, more conservative bound for size_x/size_y.
TAG_SIZE       = OUTER_TAG_SIZE

# ─────────────────────────────────────────────────────────────────────────────
# UGV LoRa link Config
# ─────────────────────────────────────────────────────────────────────────────
# The companion computer drives the ground vehicle (UGV) over a 915 MHz USB
# LoRa dongle (SB Components), mirroring lora2.py.  The UGV's lora_bridge
# node accepts newline-terminated ASCII mission commands; the ones we use:
#   * "STRAIGHT" — UGV drives straight at its configured straight_speed for
#                  its c1_travel_time (~30 s) then auto-stops.  Re-sending
#                  STRAIGHT re-arms a fresh 30 s drive from that instant.
#   * "STOP"     — UGV halts immediately and ends its mission.
# NOTE: the UGV's forward speed ("very slowly") is set entirely on the UGV
# side via the mission_controller `straight_speed` ROS parameter — it is not
# encoded in the LoRa command, so adjust it there if a slower crawl is wanted.
LORA_BAUD          = 9600
# Primary port + fallbacks scanned in order (the LoRa USB adapter enumerates
# as a ttyUSB on the companion Pi; the Pixhawk is on /dev/serial0 and is never
# probed here).  Adjust LORA_PORT if the adapter lands on a different node.
LORA_PORT          = "/dev/ttyUSB0"
LORA_FALLBACK_PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
LORA_CMD_GO        = "STRAIGHT"   # UGV: start / continue driving straight
LORA_CMD_STOP      = "STOP"       # UGV: halt and end mission

# Mission timing.
AIRBORNE_HOLD_S    = 6.0    # s — after takeoff the drone flies forward (NOT up)
                            # for this long before starting its landing phase.
                            # The UGV started its slow straight-line drive at
                            # takeoff, so the drone creeps forward to stay over
                            # the marker rather than hovering in place.
UGV_DRIVE_SECONDS  = 30.0   # s — after the drone touches down, keep the UGV
                            # driving slowly for this long, then STOP and end.

# Forward-tracking creep during AIRBORNE_HOLD.  Body-frame +x = forward (the
# drone's heading at takeoff), so this is "fly a little bit forward, not up".
# The UGV crawls at ~0.10 m/s (its straight_speed param); we creep slightly
# faster so the drone catches up to / stays over the marker that pulled ahead
# of us during the climb, then precision_land's PD takes over the fine chase.
UGV_FORWARD_SPEED   = 0.10  # m/s — the UGV's known straight-line speed
FORWARD_TRACK_SPEED = 0.15  # m/s — drone forward body-frame creep (vz held 0)

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


class NestedTagDetection:
    """One detected tag layer.  Field names match pupil_apriltags so the rest
    of this script (which reads ``tag.pose_t``, ``tag.center``, ``tag.corners``)
    is unchanged when it gets one of these instead of an AprilTag detection.

      .tag_id   — int tag ID
      .corners  — (4, 2) float32 pixel coordinates (TL, TR, BR, BL)
      .center   — (2,) float32 center pixel coordinate
      .pose_t   — (3, 1) translation vector in meters, or None if no intrinsics
      .pose_R   — (3, 3) rotation matrix, or None if no intrinsics
      .layer    — "outer" or "inner"
    """

    def __init__(self, tag_id, corners, pose_t, pose_R, layer):
        self.tag_id  = int(tag_id)
        self.corners = corners.astype(np.float32)
        self.center  = corners.mean(axis=0).astype(np.float32)
        self.pose_t  = pose_t
        self.pose_R  = pose_R
        self.layer   = layer


class _TagTrackerCache:
    """Frame-to-frame memory that briefly retains a tag after it is lost so the
    overlay does not flicker.  Ported from
    UAV/Detectors/nested_aruco_detector.py."""

    def __init__(self, max_missing_frames=4):
        self.cached = {}   # id -> (corners, missing_counter)
        self.max_missing = max_missing_frames

    def update(self, current):
        result = dict(current)
        for tid in current:
            self.cached[tid] = (current[tid], 0)
        for tid in list(self.cached.keys()):
            if tid not in current:
                corners, missing = self.cached[tid]
                if missing < self.max_missing:
                    self.cached[tid] = (corners, missing + 1)
                    result[tid] = corners
                else:
                    del self.cached[tid]
        return result


class NestedArucoDetector:
    """Inlined copy of UAV/Detectors/nested_aruco_detector.py (cv2.aruco
    backend) — kept in-file per the single, self-contained script style of
    this mission.

    Detects BOTH layers of the nested 36h11 board (OUTER id 77, INNER id 12)
    with a two-pass mask-and-redetect, then exposes ``get_tag_detection`` which
    ALWAYS prioritises the inner tag: whenever #12 is visible it is returned for
    tracking / landing, falling back to the outer tag only when the inner is
    not yet resolvable.  This is the close-range survivability that lets us
    drop the old "tag left the FOV because we got too close" fallbacks.

    Intrinsics are pulled lazily from the OAK-D calibration on the first frame
    (matching the previous AprilTagDetector), so the per-layer pose is in
    metres.
    """

    def __init__(self, calibration_handler):
        self.calibration_handler = calibration_handler
        self.camera_matrix = None
        self.dist_coeffs   = None

        self.outer_id = OUTER_TAG_ID
        self.inner_id = INNER_TAG_ID
        self._obj_points = {
            self.outer_id: self._make_obj_points(OUTER_TAG_SIZE),
            self.inner_id: self._make_obj_points(INNER_TAG_SIZE),
        }

        self._clahe      = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self._clahe_deep = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))

        self._gamma_lut                = _build_gamma_lut(_GAMMA_MODERATE)
        self._gamma_lut_strong         = _build_gamma_lut(_GAMMA_DEEP)
        self._gamma_lut_mid_dark       = _build_gamma_lut(_GAMMA_MID_DARK)
        self._gamma_lut_extreme        = _build_gamma_lut(_GAMMA_EXTREME)
        self._gamma_lut_white_mild     = _build_gamma_lut(_GAMMA_WHITE_MILD)
        self._gamma_lut_white_moderate = _build_gamma_lut(_GAMMA_WHITE_MODERATE)
        self._gamma_lut_white_strong   = _build_gamma_lut(_GAMMA_WHITE_STRONG)

        self._dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self._params     = aruco.DetectorParameters()
        self._params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._detector   = aruco.ArucoDetector(self._dictionary, self._params)

        self._cache = (
            _TagTrackerCache(NESTED_MAX_MISSING_FRAMES)
            if NESTED_MAX_MISSING_FRAMES > 0 else None
        )

        print(f"[INFO] Nested ArUco detector initialized "
              f"(outer={self.outer_id}, inner={self.inner_id})")

    @staticmethod
    def _make_obj_points(size: float) -> np.ndarray:
        """3-D corners of a tag's black square in its own frame, matching the
        ArUco corner order TL, TR, BR, BL."""
        half = size / 2.0
        return np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

    def _update_intrinsics(self, frame):
        h, w = frame.shape[:2]
        intrinsics = self.calibration_handler.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, w, h
        )
        self.camera_matrix = np.array(intrinsics, dtype=np.float64)
        self.dist_coeffs = np.array(
            self.calibration_handler.getDistortionCoefficients(
                dai.CameraBoardSocket.CAM_A),
            dtype=np.float64,
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

    def _has_all_targets(self, found: dict) -> bool:
        return self.outer_id in found and self.inner_id in found

    def _landing_target_ready(self, found: dict) -> bool:
        """True once we have corners for either landing layer (inner wins)."""
        return self.inner_id in found or self.outer_id in found

    def _select_landing_corners(self, found: dict):
        """Return (tag_id, corners) for the best available landing layer."""
        if self.inner_id in found:
            return self.inner_id, found[self.inner_id]
        if self.outer_id in found:
            return self.outer_id, found[self.outer_id]
        return None, None

    def _detection_from_corners(self, tag_id: int, corners: np.ndarray):
        """Build a NestedTagDetection with pose, or None if pose fails."""
        layer = "outer" if tag_id == self.outer_id else "inner"
        pose = self._estimate_pose(tag_id, corners)
        if pose is None:
            return None
        pose_t, pose_R = pose
        return NestedTagDetection(tag_id, corners, pose_t, pose_R, layer)

    def _detect_two_pass(self, image: np.ndarray, found: dict):
        """Pass 1 detects markers directly (usually the inner tag); pass 2
        paints over every found quad with the local background colour and
        re-detects so the outer payload decodes once the inner tag is masked
        out.  Newly found target corners are added into ``found`` keyed by id."""
        corners1, ids1, _ = self._detector.detectMarkers(image)

        if ids1 is not None:
            for i, tid in enumerate(ids1.flatten()):
                tid = int(tid)
                if tid in self._obj_points and tid not in found:
                    found[tid] = corners1[i].reshape(4, 2)

        if ids1 is None or self._has_all_targets(found):
            return

        h, w = image.shape[:2]
        masked = image.copy()
        for c in corners1:
            pts = c.reshape((-1, 2)).astype(np.float32)
            center = pts.mean(axis=0)
            ring_outer = (center + 1.35 * (pts - center)).astype(np.int32)
            ring_inner = (center + 1.10 * (pts - center)).astype(np.int32)
            ring_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(ring_mask, [ring_outer], 255)
            cv2.fillPoly(ring_mask, [ring_inner], 0)
            ring_vals = image[ring_mask == 255]
            fill_val = int(np.median(ring_vals)) if ring_vals.size else 255
            fill_poly = (center + 1.25 * (pts - center)).astype(np.int32)
            cv2.fillPoly(masked, [fill_poly], color=fill_val)

        corners2, ids2, _ = self._detector.detectMarkers(masked)
        if ids2 is not None:
            for i, tid in enumerate(ids2.flatten()):
                tid = int(tid)
                if tid in self._obj_points and tid not in found:
                    found[tid] = corners2[i].reshape(4, 2)

    def _estimate_pose(self, tag_id: int, corners: np.ndarray):
        """Per-layer pose via IPPE_SQUARE.  The gray frame is already
        undistorted in _prepare_gray, so zero distortion is passed here."""
        success, rvec, tvec = cv2.solvePnP(
            self._obj_points[tag_id],
            corners.astype(np.float32),
            self.camera_matrix,
            np.zeros(5, dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None
        pose_R, _ = cv2.Rodrigues(rvec)
        return tvec.reshape(3, 1), pose_R

    def _prepare_gray(self, frame):
        if self.camera_matrix is None:
            self._update_intrinsics(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.undistort(gray, self.camera_matrix, self.dist_coeffs)

    def detect(self, frame):
        """Return a list of NestedTagDetection (0, 1 or 2 items) — whichever of
        the outer / inner layers is currently visible."""
        if frame is None:
            return []

        gray = self._prepare_gray(frame)

        found = {}
        for variant in self._preprocess_variants(gray):
            self._detect_two_pass(variant, found)
            # Inner alone is enough for landing; stop early instead of running
            # all 27 variants hunting for the second layer (was ~seconds/frame
            # at 640×640 and blocked the control loop).
            if self.inner_id in found:
                break
            if self._has_all_targets(found):
                break

        if self._cache is not None:
            found = self._cache.update(found)

        results = []
        for tag_id, corners in found.items():
            det = self._detection_from_corners(tag_id, corners)
            if det is not None:
                results.append(det)
        return results

    def get_tag_detection(self, frame):
        """Single landing target, INNER tag prioritised over OUTER.  Returns a
        NestedTagDetection (drop-in for the old AprilTag detection) or None.

        Uses a fast path (undistorted gray first, then preprocessing variants
        with early exit) so the mission loop can run detection every tick without
        blocking for multiple seconds per frame."""
        if frame is None:
            return None

        gray = self._prepare_gray(frame)
        found = {}

        def _try_landing_target():
            merged = self._cache.update(dict(found)) if self._cache else found
            tag_id, corners = self._select_landing_corners(merged)
            if tag_id is None:
                return None
            return self._detection_from_corners(tag_id, corners)

        # Fast path — often enough at mid range without preprocessing.
        self._detect_two_pass(gray, found)
        det = _try_landing_target()
        if det is not None:
            return det

        for variant in self._preprocess_variants(gray):
            self._detect_two_pass(variant, found)
            det = _try_landing_target()
            if det is not None:
                return det
            if self.inner_id in found:
                break
            if self._landing_target_ready(found):
                break

        return None


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

        # OAK-D S2 stereo-depth altitude cache (Issue 4).
        # Written by make_pump() from the StereoDepth output queue.
        # "value" is the EMA-smoothed centre-pixel depth in metres (AGL
        # altitude when the drone is level).  "t" is wall-clock time of
        # the last valid update.  Both start None; _relative_altitude_m()
        # checks freshness and tilt before trusting this value.
        self.last_depth_alt = {
            "value": None,   # metres AGL (depth-based)
            "t":     0.0,
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

        # ── Step 5: climb monitor — altitude is verified against the
        #    AUTHORITATIVE AGL source (OAK-D stereo depth when usable, EKF
        #    fallback; see _altitude_reading).  NAV_TAKEOFF gets the airframe
        #    off the ground; once airborne we hold horizontal position and
        #    drive the FCU's z setpoint (via _altitude_setpoint_z) so the
        #    DEPTH-measured ground distance — not the EKF z — converges on
        #    the target.  The EKF-origin offset that previously left
        #    relative_alt plateauing below target (and timing out) is thus
        #    corrected: depth measures the true distance to the ground.
        start = time.time()
        liftoff_deadline = start + 6.0   # by 6 s we expect SOME vertical motion
        liftoff_seen = False
        last_relative = 0.0
        last_log = 0.0
        x_hold = None
        y_hold = None
        last_pos_cmd = 0.0
        while time.time() - start < TAKEOFF_TIMEOUT_S:
            self._drain_messages()
            if pump_fn is not None:
                pump_fn()

            now = time.time()
            cur_z = self.last_pos.get("z")
            # Liftoff is detected from the EKF (always available) — it is a
            # motion check, not an altitude-source decision.
            ekf_rel = None
            if cur_z is not None:
                ekf_rel = -(cur_z - z0)
                last_relative = ekf_rel
                if abs(ekf_rel) > 0.3:
                    liftoff_seen = True

            alt, alt_src = _altitude_reading(self)

            # Once airborne, hold XY and steer the FCU z target toward the
            # depth-measured altitude.  Position targets are withheld until
            # liftoff so they cannot fight NAV_TAKEOFF's initial climb.
            if liftoff_seen and now - last_pos_cmd >= 0.5:
                if x_hold is None and self.last_pos.get("x") is not None:
                    x_hold = self.last_pos["x"]
                    y_hold = self.last_pos["y"]
                z_set = _altitude_setpoint_z(self, meters)
                if x_hold is not None and z_set is not None:
                    self.master.mav.set_position_target_local_ned_send(
                        0,
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        TYPEMASK_POSITION_ONLY,
                        x_hold, y_hold, z_set,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0,
                    )
                    last_pos_cmd = now

            if alt is not None and now - last_log >= 0.5:
                raw_z_txt = f"{-cur_z:.2f}" if cur_z is not None else "n/a"
                print(f"[INFO] Altitude: {alt:+.2f} m [{alt_src}]  "
                      f"(raw -z = {raw_z_txt} m, anchor = {-z0:.2f} m)")
                last_log = now

            # Completion is decided on the authoritative altitude (depth when
            # usable) rather than the EKF z, and only after liftoff so an
            # on-ground depth reading cannot satisfy it.
            if (liftoff_seen and alt is not None
                    and alt >= meters - TAKEOFF_ALT_TOLERANCE_M):
                print(f"[INFO] Target altitude reached "
                      f"({alt:+.2f} m [{alt_src}], target {meters} m)")
                return

            # Drone-never-moved guard.  If 6 s after takeoff the drone has
            # not visibly climbed, the FCU almost certainly dropped the
            # takeoff (no ACK case above) — abort now rather than continue
            # with a grounded, armed drone.
            if not liftoff_seen and now > liftoff_deadline:
                print("[ERROR] Drone never lifted off — relative altitude "
                      f"{last_relative:+.2f} m after 6 s.  The FCU most "
                      "likely silently rejected NAV_TAKEOFF (EKF/GPS not "
                      "healthy or pre-arm checks failing).")
                raise RuntimeError(
                    "Drone never lifted off after NAV_TAKEOFF (no vertical "
                    "motion within 6 s)"
                )

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
        # _altitude_hold_estimate is depth-authoritative but lifts to the
        # EKF whenever the EKF reads higher, so a pegged/saturated stereo
        # depth (which plateaus near ~5 m as the airframe climbs out of
        # reliable stereo range) cannot mask an over-climb.  Bail-out path
        # keeps vz = 0 if we have no altitude reading, so the airframe is at
        # worst no worse off than the previous TRACK_VZ_HOLD behaviour.
        rel_alt, _alt_src = _altitude_hold_estimate(self)
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
            lead = min(frame_age, DESCENT_DEADRECKON_MAX_S)
            body_x -= self.last_cmd.get("vx", 0.0) * lead
            body_y -= self.last_cmd.get("vy", 0.0) * lead

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

        # Vertical command: descend toward DESCENT_TARGET_BZ, but PAUSE the
        # descent whenever the lateral error is outside the centring cone so
        # the tag cannot slide to the FOV edge (and bz inflate via slant
        # range) while we keep sinking — recentre first, then descend briskly.
        error_z = filt_z - DESCENT_TARGET_BZ
        xy_err = max(abs(filt_x), abs(filt_y))
        xy_gate = max(DESCENT_XY_ERR_HOLD, DESCENT_XY_ERR_HOLD_FRAC * filt_z)
        if error_z <= 0.0:
            # Below target altitude — let caller commit to LAND.  Never
            # command upward velocity here; we'd just chase noise.
            vz = 0.0
        elif xy_err > xy_gate:
            # Off-centre beyond the cone — hold altitude and let the XY PD
            # pull us in before sinking any further.
            vz = 0.0
        else:
            vz_cmd = max(DESCENT_MIN_VZ, DESCENT_Kp_Z * error_z)
            vz = min(vz_cmd, DESCENT_MAX_VZ)

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

    def world_to_body_xy(self, vN, vE):
        """Rotate a world-frame (NED) horizontal velocity into the body
        frame using the cached ATTITUDE.yaw.

        Same rotation as ``body_frame_velocity`` but applied to an
        arbitrary world-frame XY vector — used by the recovery loop to
        convert its world-frame position-PD command into the body-frame
        signal that ``send_velocity`` (MAV_FRAME_BODY_NED) ships.

        Returns (None, None) if yaw has not yet been received — the
        caller is expected to fall back to a hover command in that
        case rather than send a wrong-frame velocity.
        """
        yaw = self.last_att.get("yaw")
        if yaw is None:
            return None, None
        c = math.cos(yaw)
        s = math.sin(yaw)
        body_vx =  vN * c + vE * s
        body_vy = -vN * s + vE * c
        return body_vx, body_vy

    def recover_velocity_command(self, drift_snapshot, last_tag_body=None,
                                 loss_anchor=None):
        """Closed-loop position recovery toward the NED position the
        airframe was at the instant the AprilTag was lost, while
        actively returning to TAKEOFF_ALTITUDE on the vertical axis.

        ``loss_anchor`` is the (x_loss, y_loss, z_loss) Pixhawk-EKF NED
        position captured at the moment of loss.  When provided, the
        loop is:

            err_world  = (anchor_xy - cur_xy)
            v_world_xy = RECOVERY_POS_KP * err_world
                       + RECOVERY_POS_KD * (-cur_world_velocity_xy)
            v_body_xy  = R(yaw) · v_world_xy             # NED → body
            send_velocity(v_body_xy, vz_alt_loop)

        The +Kd term explicitly damps any residual world-frame velocity
        (i.e. ongoing wind drift the EKF is still measuring) — this is
        the term that prevents the old open-loop counter-drift design
        from sitting stagnant in a steady wind.  Vertical control is a
        separate P-loop on (TAKEOFF_ALTITUDE - rel_alt) using
        RECOVERY_ALT_KP / RECOVERY_MAX_VZ so the airframe converges on
        the same set altitude regardless of where in the flight it
        was lost.  ``loss_anchor.z`` is intentionally NOT used for
        vertical control: in PRECISION_LAND it would carry the descent
        altitude forward, which is the opposite of what we want.

        Backwards-compat: ``drift_snapshot`` (the body-frame velocity
        sampled at loss) and ``last_tag_body`` (the last visible body
        offset) are still accepted so the per-tick log can show the
        original drift direction and so the OAK-D BNO086 cross-check
        still has a value to compare against.  When ``loss_anchor`` is
        omitted the function falls back to the legacy open-loop
        counter-drift behaviour (only used as a degenerate fallback
        if EKF position is unavailable at the moment of loss; the
        normal call path always passes ``loss_anchor``).

        Sends a body-frame velocity (vx, vy, vz) and returns a dict
        describing the action so the caller can log it / surface it on
        the HUD:

            {
                "vx": float, "vy": float, "vz": float,
                "drift_x": float, "drift_y": float,
                "err_x": float, "err_y": float,
                "err_xy": float,
                "rel_alt": float | None,
                "alt_err": float | None,
                "source": "pos" | "ekf" | "tag_offset" | "none",
                "oak_confirm": bool,
                "scale": float,
                "in_anchor_radius": bool,
            }
        """
        self._drain_messages()

        snap_x, snap_y = drift_snapshot

        # ── Vertical loop (closed on relative altitude vs TAKEOFF_ALTITUDE) ──
        # Independent of the lateral source so a degenerate position read
        # never blocks the altitude correction.  Uses _altitude_hold_estimate
        # (depth-authoritative, but lifted to the EKF when the EKF reads
        # higher) so a pegged stereo depth cannot hide an over-climb.
        rel_alt, _alt_src = _altitude_hold_estimate(self)
        alt_err = None
        if rel_alt is None:
            vz = 0.0
        else:
            alt_err = TAKEOFF_ALTITUDE - rel_alt
            if abs(alt_err) < RECOVERY_ALT_DEADBAND_M:
                vz = 0.0
            else:
                vz_cmd = -RECOVERY_ALT_KP * alt_err
                vz = max(min(vz_cmd, RECOVERY_MAX_VZ), -RECOVERY_MAX_VZ)
            # Hard cap (user req): the relocate climb must NEVER exceed
            # TAKEOFF_ALTITUDE.  Once at/above it, forbid any commanded climb
            # (NED vz<0) so the ascend-on-loss can only ever bring us back UP
            # toward the set altitude, not overshoot above it.
            if rel_alt >= TAKEOFF_ALTITUDE and vz < 0.0:
                vz = 0.0

        # ── Lateral loop selection ──────────────────────────────────────────
        # Primary: closed-loop NED position-PD anchored at loss_anchor.
        # Fallback: legacy body-frame counter-drift on drift_snapshot
        # (used only when loss_anchor is unavailable or current EKF
        # position / yaw is degenerate this tick).
        cur_x = self.last_pos.get("x")
        cur_y = self.last_pos.get("y")
        cur_vN = self.last_pos.get("vx")
        cur_vE = self.last_pos.get("vy")
        yaw = self.last_att.get("yaw")

        err_x = 0.0
        err_y = 0.0
        err_xy = 0.0
        in_anchor_radius = False
        source = "none"
        oak_confirm = False
        scale = 1.0
        vx = 0.0
        vy = 0.0

        if (loss_anchor is not None
                and cur_x is not None and cur_y is not None
                and yaw is not None):
            # ── Position-PD path (primary) ───────────────────────────────
            anchor_x, anchor_y, _anchor_z = loss_anchor
            err_x = anchor_x - cur_x
            err_y = anchor_y - cur_y
            err_xy = math.sqrt(err_x * err_x + err_y * err_y)
            in_anchor_radius = err_xy < RECOVERY_ANCHOR_RADIUS_M
            source = "pos"

            # OAK-D BNO086 sanity check on the EKF velocity reading.
            # If the EKF reports body-frame motion (drift_snapshot above
            # deadband) but the OAK accelerometer sees nothing the two
            # IMU stacks disagree — soften the lateral command so a bad
            # EKF velocity bias can't drive the airframe sideways at
            # full authority.  When EKF reports near-zero motion we
            # ignore the cross-check entirely (no disagreement to
            # arbitrate).
            ekf_drift_mag = math.sqrt(snap_x * snap_x + snap_y * snap_y)
            if ekf_drift_mag >= RECOVERY_DRIFT_DEADBAND:
                oak_ax = self.last_oak_imu.get("ema_ax")
                oak_ay = self.last_oak_imu.get("ema_ay")
                if oak_ax is not None and oak_ay is not None:
                    oak_mag = math.sqrt(oak_ax * oak_ax + oak_ay * oak_ay)
                    oak_confirm = oak_mag >= RECOVERY_OAK_ACCEL_MIN
                    if not oak_confirm:
                        scale = RECOVERY_OAK_DISAGREE_SCALE

            # World-frame PD.  Kd term takes the NEGATIVE of the
            # current world velocity so it actively damps whatever
            # drift the EKF is still measuring (the whole point of
            # the closed-loop fix).
            vN_cmd = (RECOVERY_POS_KP * err_x
                      + RECOVERY_POS_KD * (-(cur_vN or 0.0)))
            vE_cmd = (RECOVERY_POS_KP * err_y
                      + RECOVERY_POS_KD * (-(cur_vE or 0.0)))

            # Rotate world → body, apply confidence scale, deadband,
            # min-floor, then per-axis clamp.
            body_vx, body_vy = self.world_to_body_xy(vN_cmd, vE_cmd)
            if body_vx is None:
                # Yaw vanished between the guard above and here — hover
                # rather than send a wrong-frame velocity.
                vx = 0.0
                vy = 0.0
            else:
                body_vx *= scale
                body_vy *= scale

                def _floor_then_clamp(v, axis_err):
                    if abs(axis_err) < RECOVERY_POS_DEADBAND_M:
                        return 0.0
                    if abs(v) < RECOVERY_MIN_V_XY:
                        # Sign of v_cmd is preferred; if v_cmd happens to
                        # be exactly 0 (Kp and Kd cancel), use the sign
                        # of the position error so we still close in.
                        sign_src = v if v != 0.0 else axis_err
                        v = math.copysign(RECOVERY_MIN_V_XY, sign_src)
                    return max(min(v, RECOVERY_MAX_V_XY), -RECOVERY_MAX_V_XY)

                # err_x, err_y are world-frame; for the deadband decision
                # we project them through the same yaw rotation so the
                # axis check matches the command we're about to send.
                berr_x, berr_y = self.world_to_body_xy(err_x, err_y)
                if berr_x is None:
                    berr_x, berr_y = 0.0, 0.0
                vx = _floor_then_clamp(body_vx, berr_x)
                vy = _floor_then_clamp(body_vy, berr_y)
        else:
            # ── Legacy fallback path: body-frame counter-drift ───────────
            # Reached when EKF position / yaw is unavailable AND
            # loss_anchor was therefore never captured (or capture
            # failed).  Behaviour matches the pre-closed-loop design
            # so the recovery still does *something* in that case.
            source = "ekf"
            snap_mag = max(abs(snap_x), abs(snap_y))
            if (snap_mag < RECOVERY_DRIFT_DEADBAND
                    and last_tag_body is not None):
                off_x, off_y = last_tag_body
                if (abs(off_x) >= RECOVERY_POS_DEADBAND_M
                        or abs(off_y) >= RECOVERY_POS_DEADBAND_M):
                    snap_x = -off_x
                    snap_y = -off_y
                    source = "tag_offset"
                else:
                    source = "none"

            oak_ax = self.last_oak_imu.get("ema_ax")
            oak_ay = self.last_oak_imu.get("ema_ay")
            if oak_ax is not None and oak_ay is not None:
                oak_mag = math.sqrt(oak_ax * oak_ax + oak_ay * oak_ay)
                oak_confirm = oak_mag >= RECOVERY_OAK_ACCEL_MIN

            if source == "none":
                scale = 0.0
            else:
                scale = 1.0 if oak_confirm else RECOVERY_OAK_DISAGREE_SCALE

            # Reuse RECOVERY_POS_KP as the velocity-counter gain in the
            # fallback path; the magnitude scales aren't directly
            # comparable but keeping a single tunable simplifies tuning.
            vx_raw = -RECOVERY_POS_KP * snap_x * scale
            vy_raw = -RECOVERY_POS_KP * snap_y * scale

            def _floor_then_clamp_v(v, drift_on_axis):
                if abs(drift_on_axis) < RECOVERY_DRIFT_DEADBAND or scale == 0.0:
                    return 0.0
                if abs(v) < RECOVERY_MIN_V_XY:
                    v = math.copysign(RECOVERY_MIN_V_XY,
                                      v if v != 0 else -drift_on_axis)
                return max(min(v, RECOVERY_MAX_V_XY), -RECOVERY_MAX_V_XY)

            vx = _floor_then_clamp_v(vx_raw, snap_x)
            vy = _floor_then_clamp_v(vy_raw, snap_y)

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
            "err_x": err_x, "err_y": err_y, "err_xy": err_xy,
            "rel_alt": rel_alt,
            "alt_err": alt_err,
            "source": source,
            "oak_confirm": oak_confirm,
            "scale": scale,
            "in_anchor_radius": in_anchor_radius,
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
        """Quick altitude sanity check, then proceed.

        Confirms the depth-authoritative altitude is within
        STABILIZE_ALT_TOLERANCE of ``target_alt`` for a short
        STABILIZE_HOLD_SECONDS dwell and returns immediately — a fast
        confirm rather than a long multi-criteria settle.  Lateral drift and
        horizontal speed are still measured and logged but no longer gate
        the exit (the downstream tag gate + precision-land descent actively
        control position anyway).

        Returns the (x_home, y_home) NED position captured at the end of
        stabilization — this is the "original point" the rest of the flow
        uses as the anchor for the post-loss relocate.
        """
        print("[INFO] Stabilizing — quick altitude confirm...")
        x_origin, y_origin = None, None
        ok_since = None
        start = time.time()
        last_print = 0.0
        warned_no_anchor = False
        _last_pos_cmd = 0.0  # rate-limit position-hold resends to 2 Hz

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

            # Altitude uses the authoritative AGL source (OAK-D depth when
            # usable, EKF fallback — _altitude_reading), matching the
            # convention takeoff_to_altitude now climbs to.  Absolute -cur_z
            # is only used when neither depth nor a takeoff anchor exists;
            # surface a one-time WARN so that regression is visible.
            relative_alt, alt_src = _altitude_reading(self)
            if relative_alt is None:
                if not warned_no_anchor:
                    print("[WARN] No depth altitude and no takeoff anchor — "
                          "stabilize altitude check uses absolute -z")
                    warned_no_anchor = True
                relative_alt = -cur_z
                alt_src = "EKF-abs"
            alt_err  = abs(relative_alt - target_alt)
            drift    = math.sqrt((cur_x - x_origin) ** 2 +
                                 (cur_y - y_origin) ** 2)
            hspd     = math.sqrt(cur_vx ** 2 + cur_vy ** 2)

            # Gate on altitude only — a quick sanity confirm.  drift / hspd
            # are computed for the log but no longer block the exit.
            all_ok    = alt_err  < STABILIZE_ALT_TOLERANCE

            # Active altitude + position hold — resend a POSITION target
            # every 0.5 s so the FCU's own position controller maintains
            # altitude and XY.  A velocity P-loop competed with ArduCopter's
            # internal position controller after NAV_TAKEOFF and caused the
            # drone to descend despite receiving climb commands; delegating
            # altitude control to the FCU position PID is more reliable.
            # _altitude_setpoint_z steers that z target off the depth
            # reading (and degrades exactly to takeoff_z_origin - target_alt
            # when depth is unavailable), so the hold matches the climb.
            now = time.time()
            z_hold = _altitude_setpoint_z(self, target_alt)  # absolute NED z
            if z_hold is not None:
                if now - _last_pos_cmd >= 0.5:
                    self.master.mav.set_position_target_local_ned_send(
                        0,
                        self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        TYPEMASK_POSITION_ONLY,
                        x_origin, y_origin, z_hold,
                        0, 0, 0,
                        0, 0, 0,
                        0, 0,
                    )
                    _last_pos_cmd = now
            else:
                # No altitude reading yet — safe fallback: zero velocity.
                self.send_velocity(0, 0, 0)
            if now - last_print > 0.5:
                print(f"[INFO] STABILIZE alt={relative_alt:+.2f} m [{alt_src}] "
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
    """Outline the tracked layer and mark its center.  The inner tag (#12,
    prioritised for landing) is drawn orange; the outer tag (#77) green, so the
    operator can see which layer the controller is currently locked onto."""
    layer = getattr(tag, "layer", "outer")
    color = (0, 165, 255) if layer == "inner" else (0, 255, 0)
    corners = tag.corners.astype(int)
    for i in range(4):
        cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                 color, 2)
    cv2.circle(frame, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)
    cv2.putText(frame, f"{layer.upper()} id{tag.tag_id}",
                (corners[0][0], corners[0][1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


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

    alt_text   = f"{altitude:+.2f} m" if altitude is not None else "  n/a"
    depth_alt  = state.get("depth_alt")
    depth_src  = state.get("alt_source", "EKF")
    depth_text = f"{depth_alt:.2f} m" if depth_alt is not None else "n/a"

    lines = [
        (f"PHASE   {phase}",                        color_phase),
        (f"MODE    {flightmode}",                   color_phase),
        (f"MOTORS  {'ARMED' if armed else 'DISARMED'}", color_motors),
        (f"ALT     {alt_text} [{depth_src}]",       color_white),
        (f"DEPTH   {depth_text}",                   color_white),
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

def make_pump(q_rgb, q_oak_imu, detector, controller, state, q_depth=None):
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

    ``q_depth`` (optional) is the DepthAI StereoDepth output queue (Issue 4).
    Each pump tick the latest depth frame is drained and the centre-pixel
    depth (mm → m) is EMA-smoothed into ``controller.last_depth_alt``.
    When None the depth altitude feature is simply disabled.

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
        state["cmd"]       = controller.last_cmd
        # Expose depth altitude and the source label for the overlay
        state["depth_alt"] = controller.last_depth_alt.get("value")
        # Determine which source _relative_altitude_m will currently prefer
        _d = controller.last_depth_alt
        _roll  = controller.last_att.get("roll")
        _pitch = controller.last_att.get("pitch")
        _tilt  = math.radians(DEPTH_ALT_TILT_THRESHOLD_DEG)
        _depth_usable = (
            _d.get("value") is not None
            and (time.time() - _d.get("t", 0.0)) < DEPTH_ALT_STALE_S
            and DEPTH_ALT_VALID_MIN_M <= (_d.get("value") or 0.0) <= DEPTH_ALT_VALID_MAX_M
            and _roll is not None and _pitch is not None
            and abs(_roll) <= _tilt and abs(_pitch) <= _tilt
        )
        state["alt_source"] = "DEPTH" if _depth_usable else "EKF"

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

        # 1c. Drain OAK-D stereo depth queue and update the depth-altitude
        #     cache (Issue 4).  The centre pixel of the depth frame gives
        #     the range to the surface directly below the camera in mm;
        #     we convert to metres and apply a gentle EMA to suppress
        #     frame-to-frame noise.  The safety tilt check happens inside
        #     _relative_altitude_m() — here we only cache the raw reading.
        if q_depth is not None:
            try:
                depth_pkt = q_depth.tryGet()
            except Exception:
                depth_pkt = None
            if depth_pkt is not None:
                try:
                    depth_data = depth_pkt.getFrame()
                    if depth_data is not None and depth_data.size > 0:
                        dh, dw = depth_data.shape[:2]
                        raw_mm = float(depth_data[dh // 2, dw // 2])
                        raw_m  = raw_mm / 1000.0
                        prev_d = controller.last_depth_alt.get("value")
                        if (raw_m > 0.0
                                and raw_m >= DEPTH_ALT_VALID_MIN_M
                                and raw_m <= DEPTH_ALT_VALID_MAX_M):
                            # Valid reading — apply EMA
                            if prev_d is None:
                                new_d = raw_m
                            else:
                                new_d = (DEPTH_ALT_EMA_ALPHA * prev_d
                                         + (1.0 - DEPTH_ALT_EMA_ALPHA) * raw_m)
                            controller.last_depth_alt["value"] = new_d
                            controller.last_depth_alt["t"]     = time.time()
                        # Invalid reading — hold last good value; do NOT
                        # reset to None because a single bad pixel in a
                        # dense depth frame is common and should not wipe
                        # out a recently valid altitude.
                except Exception:
                    pass   # depth read errors are non-fatal

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
# UGV LoRa link — sends mission commands to the ground vehicle over the
# 915 MHz USB LoRa dongle (ported from lora2.py).  The UGV's lora_bridge node
# reads newline-terminated ASCII; we send "STRAIGHT" to start/continue the
# slow drive and "STOP" to halt.  Opening is best-effort: if no serial module
# or no LoRa adapter is present the link runs in offline mode (every send is a
# logged no-op) so a missing dongle never aborts the flight.
# ─────────────────────────────────────────────────────────────────────────────

class UGVLoraLink:
    """Thin serial sender for UGV mission commands.  Mirrors lora2.py but is
    non-fatal on failure and scans a few candidate ports."""

    def __init__(self, port=LORA_PORT, baud=LORA_BAUD,
                 fallback_ports=None):
        self.ser = None
        self.port = None
        if not SERIAL_AVAILABLE:
            print("[WARN] pyserial unavailable — UGV LoRa link OFFLINE "
                  "(commands will be logged no-ops)")
            return
        candidates = []
        for p in [port] + list(fallback_ports or LORA_FALLBACK_PORTS):
            if p not in candidates:
                candidates.append(p)
        for p in candidates:
            try:
                ser = serial.Serial(p, baud, timeout=1)
                time.sleep(1.0)   # let the adapter settle (matches lora2.py)
                self.ser = ser
                self.port = p
                print(f"[INFO] UGV LoRa link open on {p} @ {baud} baud")
                return
            except Exception as e:
                print(f"[INFO] LoRa port {p} not available: {e}")
        print("[WARN] No LoRa adapter found on any candidate port — UGV link "
              "OFFLINE (commands will be logged no-ops)")

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, command):
        """Send a newline-terminated mission command to the UGV.  Returns True
        if it was actually written to the serial link."""
        line = command if command.endswith("\n") else command + "\n"
        if not self.connected:
            print(f"[WARN] UGV LoRa OFFLINE — would have sent: {command!r}")
            return False
        try:
            self.ser.write(line.encode("ascii"))
            print(f"[INFO] UGV LoRa TX: {command}")
            return True
        except Exception as e:
            print(f"[WARN] UGV LoRa TX failed ({command!r}): {e}")
            return False

    def close(self):
        if self.connected:
            try:
                self.ser.close()
                print("[INFO] UGV LoRa link closed")
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Commit-to-LAND helper — used by the IMU-recovery timeout in track_tag() /
# precision_land() and the final touchdown commit.  Lifted to module scope so
# callers in either phase can share it; previously it was a closure inside
# precision_land().
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
# Acquire — hover in place after STABILIZE and wait for the AprilTag.  This
# replaces the box patrol per user spec: the drone holds station at the
# takeoff anchor and runs the detector every tick until the marker shows
# up in the FOV, then hands off to TRACK.  Returns the local NED position
# at acquisition (used the same way last_known was used downstream), or
# None on timeout so the caller can fall back to plain LAND.
# ─────────────────────────────────────────────────────────────────────────────

def acquire_tag(controller, pump, state, anchor_x=None, anchor_y=None,
                timeout=None):
    """Hover in place and wait for the AprilTag to appear in the FOV.

    ``anchor_x`` / ``anchor_y`` are the NED home coordinates from
    wait_stabilized().  When provided the function runs an active GPS
    wind-correction loop (Issue 3): if the drone drifts more than
    WIND_CORRECTION_THRESHOLD_M from the anchor it commands a corrective
    body-frame velocity to return — no motor-timing estimates, purely
    physical GPS/EKF position feedback.

    An altitude P-loop (Issue 1) replaces the plain vz=0 command so
    NAV_TAKEOFF overshoot is actively corrected rather than left to float.

    ``timeout`` (s) bounds the scan; defaults to ACQUIRE_TIMEOUT_S.  The
    combined track-and-descend flow passes the short TAG_GATE_TIMEOUT_S so
    this acts as a quick visibility gate rather than a long hover.
    """
    acquire_timeout = ACQUIRE_TIMEOUT_S if timeout is None else timeout
    print(f"[INFO] Phase: ACQUIRE  (waiting up to {acquire_timeout:.0f}s "
          "for AprilTag in FOV)")
    state["phase"]     = "ACQUIRE"
    state["leg_label"] = f"ACQUIRE 0.0/{acquire_timeout:.0f}s"

    start    = time.time()
    last_log = 0.0

    while True:
        controller._drain_messages()
        if not controller.master.motors_armed():
            print("[INFO] Motors disarmed during ACQUIRE — exiting.")
            return None

        elapsed = time.time() - start
        if elapsed >= acquire_timeout:
            print(f"[WARN] ACQUIRE timeout ({acquire_timeout:.0f}s) — "
                  "tag never entered FOV")
            controller.send_velocity(0.0, 0.0, 0.0)
            state["leg_label"] = ""
            return None

        # ── Issue 1: active altitude hold ────────────────────────────────
        # Replace the plain (0,0,0) with a P-loop on altitude so any
        # NAV_TAKEOFF overshoot is driven back to TAKEOFF_ALTITUDE.
        # _altitude_hold_estimate (depth-authoritative, lifted to EKF when
        # the EKF reads higher) so a pegged stereo depth can't mask a climb.
        rel_alt_acq, _alt_src_acq = _altitude_hold_estimate(controller)
        if rel_alt_acq is not None:
            _alt_err_acq = TAKEOFF_ALTITUDE - rel_alt_acq
            if abs(_alt_err_acq) < TRACK_ALT_DEADBAND_M:
                _vz_acq = 0.0
            else:
                _vz_acq = max(min(-TRACK_Kp_Z * _alt_err_acq,
                                  TRACK_MAX_VZ), -TRACK_MAX_VZ)
        else:
            _vz_acq = 0.0

        # ── Issue 3: GPS / NED wind correction ───────────────────────────
        # If the EKF NED position is available and an anchor was supplied,
        # compute a world-frame correction velocity and rotate it into body
        # frame.  This fires only when drift exceeds
        # WIND_CORRECTION_THRESHOLD_M so small GPS noise doesn't cause
        # continuous micro-corrections.
        _vx_acq = 0.0
        _vy_acq = 0.0
        if anchor_x is not None and anchor_y is not None:
            _cur_x = controller.last_pos.get("x")
            _cur_y = controller.last_pos.get("y")
            if _cur_x is not None and _cur_y is not None:
                _err_nx = anchor_x - _cur_x
                _err_ny = anchor_y - _cur_y
                _err_mag = math.sqrt(_err_nx * _err_nx + _err_ny * _err_ny)
                if _err_mag > WIND_CORRECTION_THRESHOLD_M:
                    # Dead-band per axis before rotating
                    _err_nx = (0.0 if abs(_err_nx) < WIND_CORRECTION_DEADBAND_M
                               else _err_nx)
                    _err_ny = (0.0 if abs(_err_ny) < WIND_CORRECTION_DEADBAND_M
                               else _err_ny)
                    _vN = WIND_CORRECTION_KP * _err_nx
                    _vE = WIND_CORRECTION_KP * _err_ny
                    _bvx, _bvy = controller.world_to_body_xy(_vN, _vE)
                    if _bvx is not None:
                        _vx_acq = max(min(_bvx, WIND_CORRECTION_MAX_V),
                                      -WIND_CORRECTION_MAX_V)
                        _vy_acq = max(min(_bvy, WIND_CORRECTION_MAX_V),
                                      -WIND_CORRECTION_MAX_V)

        controller.send_velocity(_vx_acq, _vy_acq, _vz_acq)

        tag = pump(detect=True)
        if tag is not None:
            pos = controller.get_local_position()
            if pos[0] is not None:
                last_x, last_y = pos[0], pos[1]
            else:
                last_x, last_y = 0.0, 0.0
            print(f"[INFO] Tag acquired after {elapsed:.1f}s — last known "
                  f"NED=({last_x:+.2f}, {last_y:+.2f})")
            controller.send_velocity(0.0, 0.0, 0.0)
            return last_x, last_y

        state["leg_label"] = f"ACQUIRE {elapsed:.1f}/{acquire_timeout:.0f}s"

        now = time.time()
        if now - last_log > 1.0:
            drift_str = ""
            if anchor_x is not None and controller.last_pos.get("x") is not None:
                _dx = anchor_x - controller.last_pos["x"]
                _dy = anchor_y - controller.last_pos["y"]
                drift_str = (f"  drift=({_dx:+.2f},{_dy:+.2f}) m "
                             f"|{math.sqrt(_dx**2+_dy**2):.2f}| m")
            print(f"[INFO] ACQUIRE t={elapsed:.1f}/{acquire_timeout:.0f}s "
                  f"— hovering, scanning for tag{drift_str}")
            last_log = now

        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Patrol — body-frame velocity legs, with mid-patrol tag-detection abort.
# Returns (last_known_x, last_known_y) if the tag was found during patrol,
# or None if the full box was flown without seeing the tag.
# NOTE: No longer called from main() — kept here as reference for the
# previous flow.  See acquire_tag() above for the current behaviour.
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
# recover_velocity_command) — we capture the EKF NED position at the
# instant of loss and run a closed-loop position-PD back toward that
# anchor for up to RECOVERY_DURATION_S, with a separate altitude
# P-loop pulling the airframe back to TAKEOFF_ALTITUDE.  If the marker
# re-appears we resume tracking; if the window expires (or we've
# already returned to within RECOVERY_ANCHOR_RADIUS_M of the anchor
# without re-acquiring) we commit directly to LAND (the user's spec;
# SEARCH is no longer triggered from TRACK).
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
    # is lost; cleared the moment the marker re-appears.  Together they
    # describe:
    #   * loss_anchor   — Pixhawk-EKF NED position at loss; the recovery
    #                     loop closes around this.
    #   * drift_snapshot — body-frame velocity at loss; informational and
    #                     used by the OAK-D BNO086 sanity cross-check
    #                     inside recover_velocity_command.
    #   * last_tag_body — last visible body offset of the marker; legacy
    #                     fallback for the degenerate-no-EKF case.
    drift_snapshot = None    # (body_vx, body_vy) m/s at loss
    last_tag_body  = None    # (body_x,  body_y)  m at last visible frame
    loss_anchor    = None    # (x, y, z) NED at loss (Pixhawk EKF)
    loss_start     = None    # wall-clock time when loss began
    departed_anchor = False  # True once recovery has actually drifted beyond
                             # RECOVERY_ANCHOR_RADIUS_M of the loss anchor —
                             # gates the "returned to anchor" early commit so a
                             # tag lost while hovering AT the anchor doesn't
                             # trip it on the 0.5 s minimum.
    last_recovery_log = 0.0

    # ── Stale-frame EKF anchor (NEW) ────────────────────────────────────
    # Separate from loss_anchor — this fires when the tag IS detected but
    # the underlying camera frame is older than CAMERA_STALE_S.  The OLD
    # behaviour in that branch was send_velocity(0,0,0), which is body-
    # frame "stop accelerating" and does NOT hold position in wind — the
    # airframe just coasted with whatever the previous velocity command
    # established.  Flight test showed this turned every stale window
    # into a free-drift window.
    #
    # FIX: on entry to a stale window we snapshot the current Pixhawk EKF
    # NED position into stale_anchor and run the same closed-loop
    # position-PD that the tag-loss recovery uses
    # (recover_velocity_command).  As soon as a fresh frame arrives we
    # clear stale_anchor and resume normal TRACK PD.  Unlike the tag-loss
    # recovery this branch has NO timeout to LAND — repeated stale frames
    # are NOT evidence the marker is gone, just evidence the camera
    # pipeline lagged.
    stale_anchor         = None    # (x, y, z) NED at first stale frame
    stale_drift_snapshot = (0.0, 0.0)  # body-frame v at stale entry

    # Last filtered body offset returned by track_velocity_command —
    # snapshotted so the TRACK_DURATION_S timeout can compare against
    # TRACK_HANDOFF_LATERAL_M without re-running the EMA / deadband.
    last_filt_x = None
    last_filt_y = None

    while True:
        controller._drain_messages()
        if not controller.master.motors_armed():
            print("[INFO] Motors disarmed during TRACK — treating as touchdown.")
            return "TOUCHDOWN"

        elapsed = time.time() - start
        if elapsed >= TRACK_DURATION_S:
            controller.send_velocity(0.0, 0.0, 0.0)
            # Soft handoff: only proceed to PRECISION_LAND if the
            # controller actually got the lateral error inside
            # TRACK_HANDOFF_LATERAL_M.  An off-centre handoff just makes
            # PRECISION_LAND descend obliquely and almost always loses
            # the tag mid-descent (flight log: handoff at body=(-1.5,-1.5)
            # caused PRECISION_LAND to immediately enter recovery and
            # commit to LAND without descending a single metre).
            if (last_filt_x is not None
                    and last_filt_y is not None
                    and abs(last_filt_x) < TRACK_HANDOFF_LATERAL_M
                    and abs(last_filt_y) < TRACK_HANDOFF_LATERAL_M):
                print(f"[INFO] TRACK duration met ({elapsed:.1f}s) with "
                      f"lateral |x|={abs(last_filt_x):.2f}m "
                      f"|y|={abs(last_filt_y):.2f}m both < "
                      f"TRACK_HANDOFF_LATERAL_M="
                      f"{TRACK_HANDOFF_LATERAL_M:.2f}m — handing off to "
                      "PRECISION_LAND")
                return "READY"
            if last_filt_x is None or last_filt_y is None:
                last_str = "no fresh detections during TRACK"
            else:
                last_str = (f"|x|={abs(last_filt_x):.2f}m "
                            f"|y|={abs(last_filt_y):.2f}m")
            print(f"[WARN] TRACK duration met ({elapsed:.1f}s) WITHOUT "
                  f"lateral convergence ({last_str} vs handoff "
                  f"threshold {TRACK_HANDOFF_LATERAL_M:.2f}m) — aborting "
                  "to LAND in place rather than hand an off-centre "
                  "hover to PRECISION_LAND")
            return "COMMIT_LAND"

        tag = pump(detect=True)

        if tag is not None:
            # Re-acquisition (or first acquisition) — drop any recovery
            # state so the next loss starts with a fresh snapshot.
            if drift_snapshot is not None:
                print(f"[INFO] TRACK re-acquired tag after "
                      f"{time.time() - loss_start:.1f}s of IMU recovery")
                drift_snapshot = None
                loss_anchor = None
                loss_start = None
                departed_anchor = False

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
                # Stale frame — the detection is too old to feed through
                # the body-frame PD (dead-reckon compensation can't
                # account for the unmodeled wind drift between capture
                # and now).  Break the convergence streak — a frame
                # this old is not evidence of centring.
                #
                # OLD behaviour was send_velocity(0,0,0), which is a
                # body-frame "stop accelerating" command and does NOT
                # hold position in wind.  Flight test confirmed the
                # airframe free-drifted through every stale window,
                # compounding the lateral error the next fresh frame
                # then had to claw back.
                #
                # NEW behaviour: snapshot EKF NED on entry to the stale
                # window and run the closed-loop position-PD the
                # tag-loss recovery uses (recover_velocity_command).
                # That actively holds the airframe at the position
                # where we last had fresh tag data, instead of free-
                # drifting.  No timeout here — repeated stale frames
                # are NOT evidence the marker is gone.
                centred_count = 0
                if stale_anchor is None:
                    sx, sy, sz, _, _, _ = controller.get_local_position()
                    if sx is not None and sy is not None:
                        stale_anchor = (sx, sy,
                                        sz if sz is not None else 0.0)
                    bvx, bvy, _ = controller.body_frame_velocity()
                    if bvx is None or bvy is None:
                        bvx, bvy = 0.0, 0.0
                    stale_drift_snapshot = (bvx, bvy)
                    print(f"[INFO] TRACK stale frame — anchoring at NED="
                          f"{stale_anchor}, drift_snap="
                          f"({bvx:+.2f},{bvy:+.2f}) m/s")
                if stale_anchor is not None:
                    controller.recover_velocity_command(
                        stale_drift_snapshot,
                        last_tag_body=last_tag_body,
                        loss_anchor=stale_anchor,
                    )
                else:
                    # No EKF position available — fall back to the
                    # old zero-velocity behaviour so we never end up
                    # in a worse state than before.
                    controller.send_velocity(0.0, 0.0, 0.0)
                now = time.time()
                if now - last_log > 1.0:
                    print(f"[INFO] TRACK stale frame (age={frame_age:.2f}s "
                          f"> {CAMERA_STALE_S:.2f}s) — holding via EKF "
                          "anchor")
                    last_log = now
                time.sleep(0.05)
                continue

            # Fresh frame — clear any pending stale-frame anchor so the
            # next stale window starts with a fresh snapshot.
            if stale_anchor is not None:
                print("[INFO] TRACK fresh frame — clearing stale anchor")
                stale_anchor = None
                stale_drift_snapshot = (0.0, 0.0)

            filt_x, filt_y, vx, vy, vz = controller.track_velocity_command(
                body_x, body_y, body_z, frame_age=frame_age,
            )
            last_filt_x = filt_x
            last_filt_y = filt_y

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
                rel_alt, alt_src = _altitude_hold_estimate(controller)
                alt_str = (f"{rel_alt:+.2f}/{TRACK_TARGET_ALT_M:.1f}m [{alt_src}]"
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

            # First frame of loss: capture the closed-loop anchor (EKF
            # NED position at the instant of loss) AND the body-frame
            # drift snapshot.  The anchor is what the recovery actually
            # closes the loop on; the drift snapshot is informational
            # and used by the OAK-D sanity check.  Sampling later would
            # just measure our own counter command rather than the
            # original drift / position.
            if drift_snapshot is None:
                loss_start = time.time()
                bvx, bvy, _ = controller.body_frame_velocity()
                if bvx is None or bvy is None:
                    bvx, bvy = 0.0, 0.0
                drift_snapshot = (bvx, bvy)

                ax, ay, az, _, _, _ = controller.get_local_position()
                if ax is not None and ay is not None:
                    loss_anchor = (ax, ay, az if az is not None else 0.0)
                else:
                    loss_anchor = None
                print(f"[INFO] TRACK tag lost — IMU recovery snapshot "
                      f"body_v=({bvx:+.2f},{bvy:+.2f}) m/s, "
                      f"anchor_ned={loss_anchor}, last_tag_body="
                      f"{last_tag_body}")

            recovery_elapsed = time.time() - loss_start
            if recovery_elapsed > RECOVERY_DURATION_S:
                print(f"[WARN] TRACK IMU recovery exhausted "
                      f"({recovery_elapsed:.1f}s without re-acquisition) — "
                      "committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            rec = controller.recover_velocity_command(
                drift_snapshot, last_tag_body=last_tag_body,
                loss_anchor=loss_anchor,
            )

            # Track whether recovery has actually drifted clear of the
            # anchor.  At the instant of loss the airframe IS the anchor, so
            # "returned to anchor" is only meaningful once it has first left
            # the radius — otherwise a tag that flickers out while hovering
            # centred (common at TAKEOFF_ALTITUDE where the tag is small and
            # near the FOV edge) trips the early commit on the 0.5 s floor.
            if rec.get("err_xy") is not None \
                    and rec["err_xy"] > RECOVERY_ANCHOR_RADIUS_M:
                departed_anchor = True

            # Early commit-to-LAND: we've actively flown back to within
            # RECOVERY_ANCHOR_RADIUS_M of where we lost the tag and it
            # still isn't visible — this is much stronger evidence the
            # marker is genuinely gone than a stationary timer expiry,
            # so cut the recovery short instead of running the clock
            # out hovering on a tag that won't reappear.  Require at
            # least 0.5 s of recovery first, AND that recovery actually
            # had to fly back (departed_anchor) so a tag lost while already
            # centred over the anchor uses the full RECOVERY_DURATION_S
            # window before committing.
            if (rec.get("in_anchor_radius")
                    and departed_anchor
                    and recovery_elapsed > 0.5
                    and rec["source"] == "pos"):
                print(f"[INFO] TRACK recovery returned to within "
                      f"{RECOVERY_ANCHOR_RADIUS_M:.2f} m of loss anchor "
                      f"(err_xy={rec['err_xy']:.2f} m) without "
                      "re-acquiring tag — committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            state["leg_label"] = (
                f"TRACK RECOVER {recovery_elapsed:.1f}/"
                f"{RECOVERY_DURATION_S:.1f}s "
                f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},{rec['vz']:+.2f}) "
                f"err={rec['err_xy']:.2f}m src={rec['source']}"
            )

            now = time.time()
            if now - last_recovery_log > 0.5:
                alt_str = (f"{rec['rel_alt']:+.2f}/{TAKEOFF_ALTITUDE:.1f}m"
                           if rec['rel_alt'] is not None else "n/a")
                alt_err_str = (f"{rec['alt_err']:+.2f}m"
                               if rec['alt_err'] is not None else "n/a")
                print(f"[INFO] TRACK RECOVER t={recovery_elapsed:.1f}/"
                      f"{RECOVERY_DURATION_S:.1f}s  "
                      f"err=({rec['err_x']:+.2f},{rec['err_y']:+.2f})|"
                      f"{rec['err_xy']:.2f}m  "
                      f"drift=({rec['drift_x']:+.2f},{rec['drift_y']:+.2f}) "
                      f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},"
                      f"{rec['vz']:+.2f})  "
                      f"alt={alt_str} alt_err={alt_err_str}  "
                      f"src={rec['source']}  "
                      f"oak={'yes' if rec['oak_confirm'] else 'no'}  "
                      f"scale={rec['scale']:.2f}")
                last_recovery_log = now

        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Precision-landing phase.  Returns one of:
#   "TOUCHDOWN"    — motors auto-disarmed; mission complete (also returned
#                    after a final TOUCHDOWN_BODY_Z_M commit-to-LAND)
#   "COMMIT_LAND"  — IMU tag-loss recovery window expired without re-
#                    acquisition; caller commits to ArduCopter LAND mode
# ─────────────────────────────────────────────────────────────────────────────

def precision_land(controller, pump, state, tag_previously_acquired=False):
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

    Tag-loss handling (the old CLOSE-TAG "too close → LAND" handoff is gone —
    the nested board's INNER tag #12 stays resolvable at close range, so a
    loss is no longer assumed to mean "we're on top of it"):

      * BRIEF COAST: for the first TAG_COAST_S of a loss, keep flying the last
        lateral centring command (vertical paused) — a tilt/shake/blur dropout
        re-acquires in well under a second.

      * IMU RECOVERY: for losses outlasting the coast window, capture the EKF
        NED position at the moment of loss and run a closed-loop position-PD
        back to that anchor for up to RECOVERY_DURATION_S, with a separate
        altitude P-loop pulling the airframe back to TAKEOFF_ALTITUDE.  The
        OAK-D S2 BNO086 accelerometer is a sanity cross-check on the EKF
        velocity.  If the marker reappears we resume descent; if the window
        expires (or we've returned to within RECOVERY_ANCHOR_RADIUS_M of the
        anchor without re-acquiring) we commit to LAND.

    True touchdown is still committed while the tag IS visible, via the
    TOUCHDOWN_BODY_Z_M / TOUCHDOWN_HARD_FLOOR_BZ_M gate below.

    Returns one of:
      * "TOUCHDOWN"   — motors auto-disarmed; mission complete
      * "COMMIT_LAND" — IMU recovery window expired; caller commits to LAND
    """
    print("[INFO] Phase: PRECISION_LAND")
    state["phase"] = "PRECISION_LAND"
    last_tag_time = state.get("last_tag_time") or time.time()
    tag_ever_seen = tag_previously_acquired
    pl_acquire_start = time.time()
    last_body_z   = None
    last_tag_body = None      # (body_x, body_y) on last frame the tag was seen
    coast_cmd     = None      # (vx, vy) of the last descent command — re-sent
                              # during the TAG_COAST_S brief-dropout window so a
                              # tilt/shake loss keeps centring instead of
                              # immediately dropping into IMU recovery.
    last_log      = 0.0

    # IMU recovery state — see the matching block in track_tag() for
    # the full rationale.  loss_anchor + drift_snapshot are captured on
    # the first frame after loss and held constant throughout the
    # recovery window — the closed-loop position-PD inside
    # recover_velocity_command actively measures position error against
    # loss_anchor on every tick.
    drift_snapshot = None
    loss_anchor    = None
    loss_start     = None
    departed_anchor = False  # True once recovery has actually drifted beyond
                             # RECOVERY_ANCHOR_RADIUS_M of the loss anchor —
                             # gates the "returned to anchor" early commit so a
                             # tag lost while hovering AT the anchor doesn't
                             # trip it on the 0.5 s minimum.
    last_recovery_log = 0.0

    # ── Stale-frame EKF anchor (NEW — see track_tag for full rationale) ──
    # Same mechanism as in TRACK: when the camera frame is older than
    # CAMERA_STALE_S but the tag itself is still being detected, hold
    # position via recover_velocity_command anchored at the EKF NED
    # snapshot taken on first stale-frame entry, instead of letting the
    # airframe free-drift while we wait for a fresh frame.
    stale_anchor         = None
    stale_drift_snapshot = (0.0, 0.0)

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
            tag_ever_seen = True
            # Re-acquisition — clear recovery state so the next loss
            # snapshots fresh drift instead of reusing a stale one.
            if drift_snapshot is not None:
                print(f"[INFO] PRECISION_LAND re-acquired tag after "
                      f"{time.time() - loss_start:.1f}s of IMU recovery")
                drift_snapshot = None
                loss_anchor = None
                loss_start = None
                departed_anchor = False

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
                # stale data.
                #
                # OLD behaviour was send_velocity(0,0,0), which is a
                # body-frame "stop accelerating" command and does NOT
                # hold position in wind.  Flight test showed the
                # airframe free-drifted through every stale window
                # during DESCENT, exactly mirroring the TRACK regression.
                #
                # NEW behaviour: same EKF-anchored hold the tag-loss
                # recovery uses — snapshot the current NED position on
                # entry to the stale window and keep
                # recover_velocity_command pulling the airframe back to
                # it until a fresh frame arrives.  No timeout here —
                # stale frames are NOT evidence the tag is gone.
                if stale_anchor is None:
                    sx, sy, sz, _, _, _ = controller.get_local_position()
                    if sx is not None and sy is not None:
                        stale_anchor = (sx, sy,
                                        sz if sz is not None else 0.0)
                    bvx, bvy, _ = controller.body_frame_velocity()
                    if bvx is None or bvy is None:
                        bvx, bvy = 0.0, 0.0
                    stale_drift_snapshot = (bvx, bvy)
                    print(f"[INFO] DESCENT stale frame — anchoring at "
                          f"NED={stale_anchor}, drift_snap="
                          f"({bvx:+.2f},{bvy:+.2f}) m/s")
                if stale_anchor is not None:
                    controller.recover_velocity_command(
                        stale_drift_snapshot,
                        last_tag_body=last_tag_body,
                        loss_anchor=stale_anchor,
                    )
                else:
                    controller.send_velocity(0.0, 0.0, 0.0)
                now = time.time()
                if now - last_log > 1.0:
                    print(f"[INFO] DESCENT stale frame "
                          f"(age={frame_age:.2f}s > {CAMERA_STALE_S:.2f}s) "
                          "— holding via EKF anchor")
                    last_log = now
                time.sleep(0.02)
                continue

            # Fresh frame — clear any pending stale-frame anchor so the
            # next stale window starts with a fresh snapshot.
            if stale_anchor is not None:
                print("[INFO] DESCENT fresh frame — clearing stale anchor")
                stale_anchor = None
                stale_drift_snapshot = (0.0, 0.0)

            # Drive the descent ourselves.  Returns the FILTERED body
            # offset so the commit-to-LAND gate below can require
            # cm-scale lateral alignment, not the raw (noisier) offset.
            filt_x, filt_y, vx, vy, vz = controller.descent_velocity_command(
                body_x, body_y, body_z, frame_age=frame_age,
            )
            coast_cmd = (vx, vy)

            # Final-approach handoff.  We commit to LAND only when either:
            #   (a) body_z < TOUCHDOWN_BODY_Z_M AND lateral alignment is
            #       tighter than TOUCHDOWN_XY_M (single-digit cm), OR
            #   (b) body_z < TOUCHDOWN_HARD_FLOOR_BZ_M — below that floor
            #       the altitude-scaled gains can't centre anyway, so
            #       prolonging PD would waste battery on an offset we
            #       cannot drive out.
            # If body_z is between the hard floor and TOUCHDOWN_BODY_Z_M
            # with a loose lateral, descent_velocity_command's XY_ERR_HOLD
            # branch has already PAUSED the descent so the drone recentres
            # laterally before sinking past the hard floor.
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
            if not tag_ever_seen:
                acquire_elapsed = time.time() - pl_acquire_start
                if acquire_elapsed < PRECISION_LAND_ACQUIRE_S:
                    rel_alt_acq, _ = _altitude_hold_estimate(controller)
                    if rel_alt_acq is not None:
                        alt_err = TAKEOFF_ALTITUDE - rel_alt_acq
                        if abs(alt_err) < TRACK_ALT_DEADBAND_M:
                            vz_hold = 0.0
                        else:
                            vz_hold = max(
                                min(-TRACK_Kp_Z * alt_err, TRACK_MAX_VZ),
                                -TRACK_MAX_VZ,
                            )
                    else:
                        vz_hold = 0.0
                    controller.send_velocity(0.0, 0.0, vz_hold)
                    state["leg_label"] = (
                        f"PL ACQUIRE {acquire_elapsed:.1f}/"
                        f"{PRECISION_LAND_ACQUIRE_S:.0f}s"
                    )
                    now = time.time()
                    if now - last_log > 1.0:
                        print(f"[INFO] PRECISION_LAND waiting for first tag "
                              f"({acquire_elapsed:.1f}/"
                              f"{PRECISION_LAND_ACQUIRE_S:.0f}s)")
                        last_log = now
                    time.sleep(0.05)
                    continue
                print(f"[WARN] PRECISION_LAND never acquired tag within "
                      f"{PRECISION_LAND_ACQUIRE_S:.0f}s — committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            elapsed = time.time() - last_tag_time

            # (The old PRIMARY close-tag check that committed to LAND when the
            # tag was last seen below CLOSE_TAG_BODY_Z_M has been removed: with
            # the nested board the INNER tag #12 stays resolvable at close
            # range, so a loss here is treated like any other — coast briefly,
            # then IMU-recover — rather than assuming we are on top of it.)

            # Brief-dropout COAST: for the first TAG_COAST_S of a loss, keep
            # flying the last lateral centring command (vertical paused — no
            # blind descent) instead of dropping into the heavier IMU
            # recovery.  A tilt/shake/motion-blur dropout is re-acquired in
            # well under a second, so this keeps the descent tracking the tag
            # through the dropout.  Crucially we do NOT set loss_start /
            # drift_snapshot here, so the recovery window and its commit-to-LAND
            # accounting only ever start once a loss OUTLASTS the coast window —
            # intermittent quickly-re-acquired losses can no longer stack up
            # into a forced LAND.
            if elapsed < TAG_COAST_S and coast_cmd is not None:
                cvx, cvy = coast_cmd
                controller.send_velocity(cvx, cvy, 0.0)
                controller.last_cmd["lt_x"] = None
                controller.last_cmd["lt_y"] = None
                controller.last_cmd["lt_z"] = None
                now = time.time()
                if now - last_log > 0.5:
                    print(f"[INFO] DESCENT coast (tag lost {elapsed:.2f}s "
                          f"< {TAG_COAST_S:.2f}s) — holding last centring "
                          f"v=({cvx:+.2f},{cvy:+.2f},+0.00)")
                    last_log = now
                time.sleep(0.02)
                continue

            # First frame of loss: capture the closed-loop NED anchor and
            # the body-frame drift snapshot.  Same rationale as
            # track_tag — recovery_velocity_command closes on the
            # anchor every tick, drift_snapshot is informational +
            # used by the OAK-D sanity check.
            if drift_snapshot is None:
                loss_start = time.time()
                bvx, bvy, _ = controller.body_frame_velocity()
                if bvx is None or bvy is None:
                    bvx, bvy = 0.0, 0.0
                drift_snapshot = (bvx, bvy)

                ax, ay, az, _, _, _ = controller.get_local_position()
                if ax is not None and ay is not None:
                    loss_anchor = (ax, ay, az if az is not None else 0.0)
                else:
                    loss_anchor = None
                print(f"[INFO] PRECISION_LAND tag lost — IMU recovery "
                      f"snapshot body_v=({bvx:+.2f},{bvy:+.2f}) m/s, "
                      f"anchor_ned={loss_anchor}, "
                      f"last_tag_body={last_tag_body}, "
                      f"last_bz={last_body_z}")

            recovery_elapsed = time.time() - loss_start
            if recovery_elapsed > RECOVERY_DURATION_S:
                # IMU recovery window expired without re-acquiring the marker.
                # (The old TAG_TOO_CLOSE_ALT_M EKF-altitude shortcut to LAND was
                # removed along with the close-tag handoff — with the nested
                # board the inner tag #12 keeps the marker visible at close
                # range, so a recovery that genuinely times out here means the
                # marker is gone, not that we are simply on top of it.)
                print(f"[WARN] PRECISION_LAND IMU recovery exhausted "
                      f"({recovery_elapsed:.1f}s without re-acquisition) "
                      f"— total tag-loss {elapsed:.1f}s — committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            rec = controller.recover_velocity_command(
                drift_snapshot, last_tag_body=last_tag_body,
                loss_anchor=loss_anchor,
            )

            # Track whether recovery has actually drifted clear of the
            # anchor.  At the instant of loss the airframe IS the anchor, so
            # "returned to anchor" is only meaningful once it has first left
            # the radius — otherwise a tag that flickers out while hovering
            # centred trips the early commit on the 0.5 s floor.
            if rec.get("err_xy") is not None \
                    and rec["err_xy"] > RECOVERY_ANCHOR_RADIUS_M:
                departed_anchor = True

            # Early commit-to-LAND: see matching block in track_tag().
            # Same logic — if we've flown back to the loss anchor and
            # the tag still isn't visible the marker is genuinely gone.
            # Require departed_anchor first so a tag lost while already
            # centred over the anchor uses the full RECOVERY_DURATION_S
            # window before committing.
            if (rec.get("in_anchor_radius")
                    and departed_anchor
                    and recovery_elapsed > 0.5
                    and rec["source"] == "pos"):
                print(f"[INFO] PRECISION_LAND recovery returned to within "
                      f"{RECOVERY_ANCHOR_RADIUS_M:.2f} m of loss anchor "
                      f"(err_xy={rec['err_xy']:.2f} m) without "
                      "re-acquiring tag — committing to LAND")
                controller.send_velocity(0.0, 0.0, 0.0)
                return "COMMIT_LAND"

            state["leg_label"] = (
                f"PL RECOVER {recovery_elapsed:.1f}/"
                f"{RECOVERY_DURATION_S:.1f}s "
                f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},{rec['vz']:+.2f}) "
                f"err={rec['err_xy']:.2f}m src={rec['source']}"
            )

            now = time.time()
            if now - last_recovery_log > 0.5:
                alt_str = (f"{rec['rel_alt']:+.2f}/{TAKEOFF_ALTITUDE:.1f}m"
                           if rec['rel_alt'] is not None else "n/a")
                alt_err_str = (f"{rec['alt_err']:+.2f}m"
                               if rec['alt_err'] is not None else "n/a")
                print(f"[INFO] PRECISION_LAND RECOVER t="
                      f"{recovery_elapsed:.1f}/{RECOVERY_DURATION_S:.1f}s  "
                      f"err=({rec['err_x']:+.2f},{rec['err_y']:+.2f})|"
                      f"{rec['err_xy']:.2f}m  "
                      f"drift=({rec['drift_x']:+.2f},{rec['drift_y']:+.2f}) "
                      f"v=({rec['vx']:+.2f},{rec['vy']:+.2f},"
                      f"{rec['vz']:+.2f})  "
                      f"alt={alt_str} alt_err={alt_err_str}  "
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

def _altitude_reading(controller):
    """Return ``(altitude_m, source)`` where ``source`` is ``"DEPTH"`` or
    ``"EKF"``.

    Single source of truth for AGL altitude (Issue 4).  ``_relative_altitude_m``
    and the altitude-hold helpers all delegate here so the depth-vs-EKF
    selection is identical everywhere and can be logged.

    Source priority:
      1. OAK-D S2 stereo-depth centre-pixel reading — preferred when:
           * fresh (age < DEPTH_ALT_STALE_S)
           * in the credible range [DEPTH_ALT_VALID_MIN_M, DEPTH_ALT_VALID_MAX_M]
           * Pixhawk ATTITUDE confirms the drone is sufficiently level:
             |roll| <= DEPTH_ALT_TILT_THRESHOLD_DEG  AND
             |pitch| <= DEPTH_ALT_TILT_THRESHOLD_DEG
         When the drone is tilted the camera is no longer pointing straight
         down; the centre-pixel depth is NOT the AGL altitude.  We fall back
         to the Pixhawk EKF in that case rather than feed a wrong altitude
         into the control loops.
      2. Pixhawk LOCAL_POSITION_NED relative to takeoff_z_origin — used
         when depth is unavailable, stale, out-of-range, or the drone is
         tilted.  This is the original behaviour.

    ``altitude_m`` is None only when neither source is available.
    """
    # ── Try OAK-D stereo depth first ─────────────────────────────────────
    roll  = controller.last_att.get("roll")
    pitch = controller.last_att.get("pitch")
    depth_val = controller.last_depth_alt.get("value")
    depth_t   = controller.last_depth_alt.get("t", 0.0)
    _tilt_rad = math.radians(DEPTH_ALT_TILT_THRESHOLD_DEG)

    if (depth_val is not None
            and (time.time() - depth_t) < DEPTH_ALT_STALE_S
            and DEPTH_ALT_VALID_MIN_M <= depth_val <= DEPTH_ALT_VALID_MAX_M
            and roll is not None and pitch is not None
            and abs(roll)  <= _tilt_rad
            and abs(pitch) <= _tilt_rad):
        return depth_val, "DEPTH"

    # ── Fall back to Pixhawk EKF altitude ────────────────────────────────
    cur_z = controller.last_pos.get("z")
    z0    = controller.takeoff_z_origin
    if cur_z is None or z0 is None:
        return None, "EKF"
    alt = -(cur_z - z0)
    # A negative result means the EKF thinks the drone is below the
    # takeoff anchor — physically impossible during normal flight and a
    # sign of EKF divergence.  Clamp to 0 so control loops remain active
    # and drive a strong climb rather than receiving a nonsense reading.
    return max(0.0, alt), "EKF"


def _relative_altitude_m(controller):
    """Return current AGL altitude in metres, or None if unavailable.

    Thin wrapper over ``_altitude_reading`` (depth-preferred, EKF fallback);
    kept for the existing call sites that only need the value.
    """
    return _altitude_reading(controller)[0]


def _altitude_setpoint_z(controller, target_alt_m):
    """NED z setpoint (absolute, same frame as LOCAL_POSITION_NED.z) that
    drives the *authoritative* AGL altitude to ``target_alt_m`` via the FCU
    position controller.

    When depth is usable this nudges the FCU's z target so the measured
    ground distance converges on ``target_alt_m``:

        z_set = cur_z - (target_alt_m - depth)

    When depth is unavailable/tilted/stale it degrades *exactly* to the
    original EKF-frame hold (``takeoff_z_origin - target_alt_m``), because
    with the EKF altitude ``alt = -(cur_z - z0)`` the expression collapses
    to ``z0 - target_alt_m``.  Delegating the inner loop to the FCU position
    PID (rather than a companion velocity loop) mirrors the lesson baked into
    wait_stabilized — a velocity P-loop fought ArduCopter's controller after
    NAV_TAKEOFF and sank the airframe.

    Returns None when neither current z nor an altitude reading is available.
    """
    cur_z = controller.last_pos.get("z")
    alt, _src = _altitude_reading(controller)
    if cur_z is None or alt is None:
        return None
    return cur_z - (target_alt_m - alt)


def _altitude_hold_estimate(controller):
    """Altitude estimate (metres, source) for the TRACK / IMU-RECOVER hold.

    Same depth-authoritative reading as _altitude_reading, but it is never
    allowed to report LOWER than the EKF-relative altitude.  The OAK-D S2
    stereo depth loses disparity resolution past ~5 m (low-disparity far
    pixels) and tends to peg/saturate near the target while the airframe
    actually climbs higher — a pegged depth would leave the vertical loop
    reading "at target" and commanding vz≈0 while the drone drifts up
    unchecked (real flight: alt stuck ~5.0 m while it climbed and the tag
    left the FOV).  Taking the max means a genuine climb detected by the EKF
    always wins, so the loop can only ever be MORE eager to descend toward
    target — never to climb above it.  Returns (None, "EKF") only when no
    source is available.
    """
    depth_alt, depth_src = _altitude_reading(controller)
    cur_z = controller.last_pos.get("z")
    z0    = controller.takeoff_z_origin
    ekf_alt = None
    if cur_z is not None and z0 is not None:
        ekf_alt = max(0.0, -(cur_z - z0))
    if depth_alt is None:
        return ekf_alt, "EKF"
    if ekf_alt is not None and ekf_alt > depth_alt:
        return ekf_alt, "EKF>DEPTH"
    return depth_alt, depth_src


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

    detector = NestedArucoDetector(calibration)

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

        # ── OAK-D S2 stereo depth for altitude (Issue 4) ──────────────────
        # The OAK-D S2 has two grayscale stereo cameras (CAM_B = left,
        # CAM_C = right) in addition to the centre colour camera (CAM_A).
        # When the device is mounted face-down on the UAV all three cameras
        # point downward, so the stereo disparity at the centre pixel gives
        # the slant range to the ground — i.e. the AGL altitude (when the
        # drone is level).
        #
        # We use HIGH_ACCURACY preset with left-right check enabled.
        # Subpixel is left off to keep CPU load manageable; the EMA in
        # make_pump further smooths frame-to-frame variance.
        #
        # If either camera or the StereoDepth node fails to create (e.g.
        # on a non-S2 OAK-D variant that lacks one socket), we catch the
        # exception and set q_depth=None so the rest of the mission can
        # continue using EKF altitude exclusively.
        q_depth = None
        try:
            left_cam = pipeline.create(dai.node.Camera)
            left_cam.build(dai.CameraBoardSocket.CAM_B)
            right_cam = pipeline.create(dai.node.Camera)
            right_cam.build(dai.CameraBoardSocket.CAM_C)

            stereo = pipeline.create(dai.node.StereoDepth)
            # PresetType was removed/renamed in some depthai builds; fall back
            # to manual config so the node is always fully initialised.
            if hasattr(dai.node.StereoDepth, 'PresetType'):
                stereo.setDefaultProfilePreset(
                    dai.node.StereoDepth.PresetType.HIGH_ACCURACY)
            else:
                stereo.initialConfig.setConfidenceThreshold(200)
                stereo.setRectifyEdgeFillColor(0)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(False)

            left_out  = left_cam.requestOutput(
                size=DEPTH_STEREO_RES,
                type=dai.ImgFrame.Type.GRAY8,
                fps=30,
            )
            right_out = right_cam.requestOutput(
                size=DEPTH_STEREO_RES,
                type=dai.ImgFrame.Type.GRAY8,
                fps=30,
            )
            left_out.link(stereo.left)
            right_out.link(stereo.right)

            q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
            print("[INFO] OAK-D S2 stereo depth pipeline ready "
                  f"(resolution {DEPTH_STEREO_RES}, tilt guard "
                  f"±{DEPTH_ALT_TILT_THRESHOLD_DEG}°)")
        except Exception as _depth_ex:
            print(f"[WARN] Could not set up stereo depth pipeline: {_depth_ex} "
                  "— altitude will use Pixhawk EKF only")
            q_depth = None

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
            "depth_alt":      None,   # OAK-D stereo depth altitude (Issue 4)
            "alt_source":     "EKF",  # "DEPTH" when depth is in use
            "leg_label":      "",
            "tag_visible":    False,
            "last_tag_time":  0.0,
            "time_lost":      0.0,
            "frame":          None,
            "last_tag":       None,
            "cmd":            controller.last_cmd,
        }
        pump = make_pump(q_rgb, q_oak_imu, detector, controller, state,
                         q_depth=q_depth)

        # Pump for ~0.5 s so the OpenCV window is on-screen with a phase
        # label BEFORE we touch the FCU.
        for _ in range(10):
            pump()
            time.sleep(0.05)

        # Open the UGV LoRa link now (before arming) so a missing dongle is
        # surfaced on the ground rather than mid-flight.  Offline mode is
        # non-fatal — the flight still runs, the UGV just won't be commanded.
        ugv = UGVLoraLink()

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
            # Defensive: make sure the UGV is told to STOP even though we never
            # got far enough to start it driving.
            ugv.send(LORA_CMD_STOP)
            ugv.close()
            controller.change_flight_mode("LAND")
            for _ in range(20):
                pump()
                time.sleep(0.1)
            raise SystemExit(1)

        # ── UGV GO ─────────────────────────────────────────────────────────
        # The drone is now airborne — start the ground vehicle moving (slowly;
        # the crawl speed is set on the UGV side via its straight_speed param).
        ugv.send(LORA_CMD_GO)

        try:
            # ── Stabilization (quick altitude confirm + home anchor) ────────
            print("[INFO] Phase: STABILIZE")
            state["phase"] = "STABILIZE"
            x_home, y_home = controller.wait_stabilized(
                TAKEOFF_ALTITUDE, pump_fn=pump,
            )
            print(f"[INFO] Home anchor captured: "
                  f"({x_home:+.2f}, {y_home:+.2f})")

            # ── Forward tracking creep ──────────────────────────────────────
            # The UGV started its slow straight-line drive at takeoff, so
            # instead of hovering in place the drone flies FORWARD (body-frame
            # +x, vz held at 0 — "a little bit forward, not up") for
            # AIRBORNE_HOLD_S so it stays over / catches up to the marker that
            # pulled ahead during the climb.  We pump the detector each tick so
            # the nested marker shows up on the HUD as it comes into view, then
            # hand off to precision_land whose PD takes over the fine chase.
            print(f"[INFO] Phase: FORWARD_TRACK ({AIRBORNE_HOLD_S:.0f}s @ "
                  f"{FORWARD_TRACK_SPEED:.2f} m/s forward)")
            state["phase"] = "FORWARD_TRACK"
            hold_start = time.time()
            while time.time() - hold_start < AIRBORNE_HOLD_S:
                controller.send_velocity(FORWARD_TRACK_SPEED, 0.0, 0.0)
                remaining = AIRBORNE_HOLD_S - (time.time() - hold_start)
                state["leg_label"] = f"FWD {remaining:.1f}s"
                pump(detect=True)
                time.sleep(0.05)

            # ── Acquire nested marker before descending ─────────────────────
            # FORWARD_TRACK creeps over the UGV but does not gate on detection.
            # Hover at the home anchor with wind correction until the nested
            # board (#12 inner / #77 outer) is visible — same as the old
            # ACQUIRE → TRACK handoff, but we go straight into precision_land.
            acq = acquire_tag(
                controller, pump, state,
                anchor_x=x_home, anchor_y=y_home,
                timeout=ACQUIRE_TIMEOUT_S,
            )
            if acq is None:
                print("[WARN] AprilTag not acquired — falling back to LAND "
                      "at current position")
                state["phase"] = "TOUCHDOWN"
                commit_to_land(
                    controller, pump,
                    "AprilTag never acquired after forward track",
                )
            else:
                # ── Combined track-and-descend (PRECISION_LAND) ─────────────
                # The drone centres over the nested marker (inner tag #12 is
                # prioritised, so it stays locked on as the outer tag overflows
                # the FOV at close range) AND descends toward it in one
                # continuous motion.  All recovery / COMMIT_LAND / touchdown
                # handling lives inside precision_land.
                result = precision_land(
                    controller, pump, state,
                    tag_previously_acquired=True,
                )
                if result == "COMMIT_LAND":
                    state["phase"] = "TOUCHDOWN"
                    commit_to_land(
                        controller, pump,
                        "PRECISION_LAND IMU recovery exhausted — final commit",
                    )
                else:
                    state["phase"] = "TOUCHDOWN"

            # ── Post-landing UGV drive ──────────────────────────────────────
            # The drone is down on the (moving) vehicle.  Tell the UGV to keep
            # driving slowly for UGV_DRIVE_SECONDS, then STOP and end the
            # mission for both vehicles.  Re-sending STRAIGHT re-arms a fresh
            # drive window from this instant.
            print(f"[INFO] Touchdown — commanding UGV to continue for "
                  f"{UGV_DRIVE_SECONDS:.0f}s")
            state["phase"] = "UGV_DRIVE"
            ugv.send(LORA_CMD_GO)
            drive_start = time.time()
            while time.time() - drive_start < UGV_DRIVE_SECONDS:
                remaining = UGV_DRIVE_SECONDS - (time.time() - drive_start)
                state["leg_label"] = f"UGV DRIVE {remaining:.1f}s"
                pump()
                time.sleep(0.1)

            print("[INFO] UGV drive window complete — commanding STOP")
            state["phase"] = "MISSION_COMPLETE"
            ugv.send(LORA_CMD_STOP)
        finally:
            # Whatever happened above (clean finish OR an exception), make sure
            # the UGV is halted and the serial link is released.
            ugv.send(LORA_CMD_STOP)
            ugv.close()

        # Final pump so the very last HUD frame is visible briefly before
        # window teardown.
        for _ in range(20):
            pump()
            time.sleep(0.05)

cv2.destroyAllWindows()
