import cv2
import numpy as np
import time
import math
from multiprocessing import shared_memory
from pymavlink import mavutil


# -----------------------
# VIO & SLAM Settings
# -----------------------
MAX_CORNERS = 300
QUALITY_LEVEL = 0.01
MIN_DISTANCE = 10
LK_WIN_SIZE = (21, 21)
LK_MAX_LEVEL = 3

MIN_PNP_POINTS = 17
DEPTH_MIN_M = 0.20
DEPTH_MAX_M = 12.0
REDETECT_EVERY = 10

# Loop closure (SLAM)
ENABLE_LOOP = True
KEYFRAME_INTERVAL = 30
LOOP_CHECK_INTERVAL = 0.5
MIN_LOOP_SEPARATION = 10
MATCH_THRESHOLD = 45
MAX_KEYFRAMES = 300
MAX_MATCH_CANDIDATES = 80
ORB_NFEATURES = 400
ORB_SCALE = 0.5

# Soft drift correction
ENABLE_SOFT_CORRECTION = True
SOFT_CORR_ALPHA = 0.15
SOFT_CORR_COOLDOWN = 1.0
MIN_DRIFT_TO_CORRECT_M = 0.20
MAX_CORR_STEP_M = 1.0

# -----------------------
# Helper Functions
# -----------------------
def wrap_deg180(a: float) -> float:
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def wrap_rad_pi(angle_rad: float) -> float:
    """Wraps an angle in radians to the strictly required [-pi, pi] range."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

def rotmat_to_yaw_deg(R: np.ndarray) -> float:
    return wrap_deg180(math.degrees(math.atan2(R[1, 0], R[0, 0])))

def clamp_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    if n <= 1e-9 or n <= max_norm:
        return vec
    return vec * (max_norm / n)

def vo_pose_to_ned(camera_x, camera_y, camera_z, yaw_offset_rad):
    """
    1. Swaps OpenCV coordinates to NED for a DOWNWARD-facing camera.
       (Assumes the top of the camera image points to the front of the drone).
    2. Rotates the 2D grid to align with the Pixhawk's true magnetic North.
    """
    # --- THE FIX: Downward Camera Geometry ---
    ned_x_unaligned = -camera_y  # Moving towards top of image = Forward (North)
    ned_y_unaligned = camera_x   # Moving towards right of image = Right (East)
    ned_z_final     = camera_z   # Moving straight out of lens = Down

    # Rotate the grid using the magnetic yaw offset
    c = math.cos(yaw_offset_rad)
    s = math.sin(yaw_offset_rad)

    ned_x_final = (ned_x_unaligned * c) - (ned_y_unaligned * s)
    ned_y_final = (ned_x_unaligned * s) + (ned_y_unaligned * c)

    return ned_x_final, ned_y_final, ned_z_final

# -----------------------
# SLAM (Loop Closure) Class
# -----------------------
class LoopClosureORB:
    def __init__(self):
        self.orb = cv2.ORB_create(nfeatures=ORB_NFEATURES)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.keyframes = []
        self.last_check_wall = 0.0

    @staticmethod
    def _to_gray(frame: np.ndarray) -> np.ndarray:
        if frame is None:
            return None
        if len(frame.shape) == 2:
            return frame
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _prep(self, frame: np.ndarray) -> np.ndarray:
        gray = self._to_gray(frame)
        if gray is None:
            return None
        if ORB_SCALE != 1.0:
            new_w = max(1, int(gray.shape[1] * ORB_SCALE))
            new_h = max(1, int(gray.shape[0] * ORB_SCALE))
            return cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_AREA)
        return gray

    def add_keyframe(self, frame: np.ndarray, pose_xyz: np.ndarray, frame_id: int, t_sec: float):
        small = self._prep(frame)
        if small is None:
            return
        _, des = self.orb.detectAndCompute(small, None)
        if des is None or len(des) == 0:
            return

        self.keyframes.append({
            "id": frame_id,
            "t_sec": t_sec,
            "des": des,
            "pose": pose_xyz.copy(),
        })

        if len(self.keyframes) > MAX_KEYFRAMES:
            self.keyframes.pop(0)

    def check_loop(self, frame: np.ndarray, current_pose_xyz: np.ndarray, frame_id: int):
        now = time.time()
        if now - self.last_check_wall < LOOP_CHECK_INTERVAL:
            return None
        self.last_check_wall = now

        if len(self.keyframes) < (MIN_LOOP_SEPARATION + 1):
            return None

        small = self._prep(frame)
        if small is None:
            return None
        _, des = self.orb.detectAndCompute(small, None)
        if des is None or len(des) == 0:
            return None

        candidates = self.keyframes[:-MIN_LOOP_SEPARATION]
        if len(candidates) > MAX_MATCH_CANDIDATES:
            candidates = candidates[-MAX_MATCH_CANDIDATES:]

        best = None
        best_score = 0

        for kf in candidates:
            if frame_id - kf["id"] < MIN_LOOP_SEPARATION:
                continue
            try:
                matches = self.bf.match(kf["des"], des)
                score = len(matches)
                if score > best_score:
                    best_score = score
                    best = kf
            except Exception:
                continue

        if best is None or best_score < MATCH_THRESHOLD:
            return None

        drift_vec = best["pose"] - current_pose_xyz
        drift_m = float(np.linalg.norm(drift_vec))

        return {
            "matched_kf_id": best["id"],
            "score": best_score,
            "drift_m": drift_m,
            "kf_time": best["t_sec"],
            "matched_pose": best["pose"].copy(),
        }

# -----------------------
# VIO Tracking Class
# -----------------------
class VO_LK:
    def __init__(self, K: np.ndarray):
        self.K = K.astype(np.float64)
        self.dist = np.zeros((4, 1), dtype=np.float64)
        self.T_w_c = np.eye(4, dtype=np.float64)

        self.prev_gray = None
        self.prev_depth = None
        self.prev_pts = None

        self.status = "INIT"
        self.num_tracked = 0
        self.num_used_pnp = 0
        self.inliers = 0
        self.frame_idx = 0
        self._last_corr_wall = 0.0

    def _detect(self, gray):
        return cv2.goodFeaturesToTrack(
            gray,
            maxCorners=MAX_CORNERS,
            qualityLevel=QUALITY_LEVEL,
            minDistance=MIN_DISTANCE,
            blockSize=7,
            useHarrisDetector=False,
        )

    def process(self, gray, depth_mm):
        self.frame_idx += 1
        W, H = gray.shape[1], gray.shape[0]

        if self.prev_gray is None or self.prev_depth is None or self.prev_pts is None:
            # FIX: Force a physical memory copy!
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = self._detect(gray)
            self.status = "WARMUP"
            return

        if self.prev_pts is None or len(self.prev_pts) < MIN_PNP_POINTS:
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = self._detect(gray)
            self.status = "REDETECT"
            return

        next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=LK_WIN_SIZE,
            maxLevel=LK_MAX_LEVEL,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if next_pts is None or st is None:
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = self._detect(gray)
            self.status = "LK_FAIL"
            return

        st = st.reshape(-1)
        prev_good = self.prev_pts[st == 1].reshape(-1, 2)
        curr_good = next_pts[st == 1].reshape(-1, 2)
        self.num_tracked = len(prev_good)

        if self.num_tracked < MIN_PNP_POINTS:
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = self._detect(gray)
            self.status = f"LOW_TRACK({self.num_tracked})"
            return

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        obj_pts, img_pts = [], []
        for (u0, v0), (u1, v1) in zip(prev_good, curr_good):
            x0, y0 = int(round(u0)), int(round(v0))
            if not (0 <= x0 < W and 0 <= y0 < H):
                continue

            z_m = float(self.prev_depth[y0, x0]) / 1000.0
            if z_m < DEPTH_MIN_M or z_m > DEPTH_MAX_M:
                continue

            X = (u0 - cx) * z_m / fx
            Y = (v0 - cy) * z_m / fy
            obj_pts.append([X, Y, z_m])
            img_pts.append([u1, v1])

        self.num_used_pnp = len(obj_pts)
        if self.num_used_pnp < MIN_PNP_POINTS:
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = curr_good.reshape(-1, 1, 2).astype(np.float32)
            self.status = f"DEPTH_FILTER({self.num_used_pnp})"
            return

        obj_pts = np.asarray(obj_pts, dtype=np.float64)
        img_pts = np.asarray(img_pts, dtype=np.float64)

        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            obj_pts, img_pts, self.K, self.dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=3.0,
            confidence=0.999,
            iterationsCount=150
        )
        if not ok or inl is None or len(inl) < 12:
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = curr_good.reshape(-1, 1, 2).astype(np.float32)
            self.status = "PNP_FAIL"
            return

        self.inliers = int(len(inl))
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3, 1)

        R_inv = R.T
        t_inv = -R_inv @ t

        T_prev_cur = np.eye(4, dtype=np.float64)
        T_prev_cur[:3, :3] = R_inv
        T_prev_cur[:3, 3:] = t_inv

        self.T_w_c = self.T_w_c @ T_prev_cur
        self.status = "TRACKING"

        if (self.frame_idx % REDETECT_EVERY) == 0:
            pts = self._detect(gray)
            self.prev_pts = pts if pts is not None and len(pts) >= MIN_PNP_POINTS else curr_good.reshape(-1, 1, 2).astype(np.float32)
        else:
            self.prev_pts = curr_good.reshape(-1, 1, 2).astype(np.float32)

        # FIX: The final copy assignment at the end of a successful loop
        self.prev_gray = gray.copy()
        self.prev_depth = depth_mm.copy()

    def apply_soft_correction(self, target_pose_xyz: np.ndarray):
        if not ENABLE_SOFT_CORRECTION: return None
        now = time.time()
        if now - self._last_corr_wall < SOFT_CORR_COOLDOWN: return None

        cur = self.T_w_c[:3, 3].reshape(3)
        drift = target_pose_xyz.reshape(3) - cur
        drift_mag = float(np.linalg.norm(drift))

        if drift_mag < MIN_DRIFT_TO_CORRECT_M: return None

        step = clamp_norm(drift, MAX_CORR_STEP_M)
        corr = step * SOFT_CORR_ALPHA
        self.T_w_c[:3, 3] = (cur + corr).reshape(3)
        self._last_corr_wall = now
        return True

    def pose(self):
        p = self.T_w_c[:3, 3].copy()
        yaw_vis = rotmat_to_yaw_deg(self.T_w_c[:3, :3])
        return p, yaw_vis


# -----------------------
# Main Process Function
# -----------------------
def calculatevioslam_updateposition(camera_frame_mutex, uart_tx_mutex, camera_calibration_mutex, yaw_mutex, position_mutex):
    W, H = 640, 400
    
    # Let the broadcaster initialize the RAM first
    time.sleep(4) 

    # --- 1. MEMORY SETUP ---
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_calib = shared_memory.SharedMemory(name="oak_calib")
    shm_yaw = shared_memory.SharedMemory(name="pixhawk_yaw")
    shm_x_coord = shared_memory.SharedMemory(name="pixhawk_x_coord")
    shm_y_coord = shared_memory.SharedMemory(name="pixhawk_y_coord")
    shm_z_coord = shared_memory.SharedMemory(name="pixhawk_z_coord")

    shared_calib = np.ndarray((3, 3), dtype=np.float64, buffer=shm_calib.buf)
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_yaw = np.ndarray((1,), dtype=np.float64, buffer=shm_yaw.buf)
    shared_x_coord = np.ndarray((1,), dtype=np.float64, buffer=shm_x_coord.buf)
    shared_y_coord = np.ndarray((1,), dtype=np.float64, buffer=shm_y_coord.buf)
    shared_z_coord = np.ndarray((1,), dtype=np.float64, buffer=shm_z_coord.buf)

    local_calib = np.zeros((3, 3), dtype=np.float64)
    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_gray = np.zeros((H, W), dtype=np.uint8)
    local_depth = np.zeros((H, W), dtype=np.uint16)
    
    # We keep a copy of the previous frame specifically to check if the memory updated
    last_processed_gray = np.zeros((H, W), dtype=np.uint8)

    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("[VIO] Connected to Shared Memory. Booting Algorithm...")

    initial_yaw_rad = 0.0
    with yaw_mutex:
        initial_yaw_rad = shared_yaw[0]
        print(f"[VIO] Initial Yaw from Shared Memory: {initial_yaw_rad} radians")

    # --- 2. VIO INIT ---
    # IMPORTANT: Update this matrix with the actual K matrix printed out in your terminal
    # when you originally ran vo_full_vers3.py! This is a generic OAK-D placeholder.
    with camera_calibration_mutex:
        np.copyto(local_calib, shared_calib)
    camera_matrix_K = local_calib.copy()

    vo = VO_LK(K=camera_matrix_K)
    loop = LoopClosureORB() if ENABLE_LOOP else None
    
    t0 = time.time()
    frame_id = 0

    print("[VIO] Algorithm running. Calculating poses...\n")

    while True:
        # --- CRITICAL SECTION: RAM COPY ---
        with camera_frame_mutex:
            np.copyto(local_rgb, shared_rgb)
            np.copyto(local_gray, shared_gray)
            np.copyto(local_depth, shared_depth)

        # Performance saving step: Did the memory actually change? 
        # If not, the broadcaster hasn't pushed a new frame yet. Skip the loop so we don't 
        # run the heavy math twice on the exact same picture.
        if np.array_equal(local_gray, last_processed_gray):
            time.sleep(0.005) # Rest the CPU for 5ms and check again
            continue
            
        np.copyto(last_processed_gray, local_gray) # Save it for the next check

        # --- OUTSIDE THE LOCK: HEAVY MATH ---
        t_sec = time.time() - t0

        # Run Odometry Tracker
        vo.process(local_gray, local_depth)
        pos, yaw_vis = vo.pose()

        # Run Loop Closure SLAM
        if ENABLE_LOOP and loop is not None and vo.status == "TRACKING":
            if (frame_id % KEYFRAME_INTERVAL) == 0:
                loop.add_keyframe(local_rgb, pos, frame_id, t_sec)

            info = loop.check_loop(local_rgb, pos, frame_id)
            if info is not None:
                vo.apply_soft_correction(info["matched_pose"])
                # Refresh position after correction
                pos, yaw_vis = vo.pose()

        frame_id += 1

        # --- UART TRANSMIT CRITICAL SECTION ---
        # We only print/transmit if we actually have tracking data
        if vo.status == "TRACKING":
            aligned_x, aligned_y, aligned_z = vo_pose_to_ned(pos[0], pos[1], pos[2], initial_yaw_rad)
            aligned_yaw_rad = initial_yaw_rad + math.radians(yaw_vis)
            aligned_yaw_rad = wrap_rad_pi(aligned_yaw_rad)

            time_usec = int(time.time() * 1e6)
            with uart_tx_mutex:
                master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
                master.target_system = 1 # Send messages to system 1(drone/vehicle #1)
                master.target_component = 1 # Send messages to flight controller "autopilot"
                master.mav.vision_position_estimate_send(time_usec, aligned_x, aligned_y, aligned_z, 0.0, 0.0, aligned_yaw_rad)
                print(f"[UART TX MOCK] ALIGNED NED | North(X):{aligned_x:+.2f}m, East(Y):{aligned_y:+.2f}m, Down(Z):{aligned_z:+.2f}m, Yaw: {aligned_yaw_rad} Rads, Time: {time_usec}us")
                master.close()
            with position_mutex:
                shared_x_coord[0] = aligned_x
                shared_y_coord[0] = aligned_y
                shared_z_coord[0] = aligned_z

        else:
            print(f"VIO LOST. Status: {vo.status}")