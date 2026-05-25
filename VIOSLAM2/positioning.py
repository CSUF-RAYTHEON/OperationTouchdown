import cv2
import numpy as np
import time
import math
import multiprocessing as mp
from VIOSLAM2.broadcaster import broadcaster
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
DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 15.0
REDETECT_EVERY = 10

# Loop closure (SLAM)
KEYFRAME_INTERVAL = 30
LOOP_CHECK_INTERVAL = 0.5
MIN_LOOP_SEPARATION = 10
MATCH_THRESHOLD = 45
MAX_KEYFRAMES = 300
MAX_MATCH_CANDIDATES = 80
ORB_NFEATURES = 400
ORB_SCALE = 0.5

# Soft drift correction
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
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

def clamp_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    if n <= 1e-9 or n <= max_norm:
        return vec
    return vec * (max_norm / n)

def vo_step_to_ned(rel_x, rel_y, rel_z, live_yaw_rad):
    """
    Takes the relative camera movement step and instantly aligns it 
    to the global magnetic NED grid using the drone's live compass.
    Camera Geometry: Top of image (-Y) is Forward.
    """
    step_n_unaligned = -rel_y  
    step_e_unaligned = rel_x   
    step_d_final     = rel_z   

    c = math.cos(live_yaw_rad)
    s = math.sin(live_yaw_rad)

    step_n_final = (step_n_unaligned * c) - (step_e_unaligned * s)
    step_e_final = (step_n_unaligned * s) + (step_e_unaligned * c)

    return step_n_final, step_e_final, step_d_final

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
        if small is None: return
        
        _, des = self.orb.detectAndCompute(small, None)
        if des is None or len(des) == 0: return

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
        if now - self.last_check_wall < LOOP_CHECK_INTERVAL: return None
        self.last_check_wall = now

        if len(self.keyframes) < (MIN_LOOP_SEPARATION + 1): return None

        small = self._prep(frame)
        if small is None: return None
        
        _, des = self.orb.detectAndCompute(small, None)
        if des is None or len(des) == 0: return None

        candidates = self.keyframes[:-MIN_LOOP_SEPARATION]
        if len(candidates) > MAX_MATCH_CANDIDATES:
            candidates = candidates[-MAX_MATCH_CANDIDATES:]

        best = None
        best_score = 0

        for kf in candidates:
            if frame_id - kf["id"] < MIN_LOOP_SEPARATION: continue
            try:
                matches = self.bf.match(kf["des"], des)
                score = len(matches)
                if score > best_score:
                    best_score = score
                    best = kf
            except Exception:
                continue

        if best is None or best_score < MATCH_THRESHOLD: return None

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
        
        # Absolute Global Tallies
        self.global_north = 0.0
        self.global_east = 0.0
        self.global_down = 0.0

        self.prev_gray = None
        self.prev_depth = None
        self.prev_pts = None

        self.status = "INIT"
        self.num_tracked = 0
        self.frame_idx = 0
        self._last_corr_wall = 0.0

    def _detect(self, gray):
        return cv2.goodFeaturesToTrack(
            gray, maxCorners=MAX_CORNERS, qualityLevel=QUALITY_LEVEL,
            minDistance=MIN_DISTANCE, blockSize=7, useHarrisDetector=False
        )

    def process(self, gray, depth_mm, live_yaw_rad):
        self.frame_idx += 1
        W, H = gray.shape[1], gray.shape[0]

        if self.prev_gray is None or self.prev_depth is None or self.prev_pts is None:
            self.prev_gray = gray.copy()
            self.prev_depth = depth_mm.copy()
            self.prev_pts = self._detect(gray)
            self.status = "WARMUP"
            return

        next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=LK_WIN_SIZE, maxLevel=LK_MAX_LEVEL,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        if next_pts is None or st is None:
            self.status = "LK_FAIL"
            return

        st = st.reshape(-1)
        prev_good = self.prev_pts[st == 1].reshape(-1, 2)
        curr_good = next_pts[st == 1].reshape(-1, 2)
        self.num_tracked = len(prev_good)

        if self.num_tracked < MIN_PNP_POINTS:
            self.status = f"LOW_TRACK({self.num_tracked})"
            return

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        obj_pts, img_pts = [], []
        for (u0, v0), (u1, v1) in zip(prev_good, curr_good):
            x0, y0 = int(round(u0)), int(round(v0))
            if not (0 <= x0 < W and 0 <= y0 < H): continue

            z_m = float(self.prev_depth[y0, x0]) / 1000.0
            if z_m < DEPTH_MIN_M or z_m > DEPTH_MAX_M: continue

            X = (u0 - cx) * z_m / fx
            Y = (v0 - cy) * z_m / fy
            obj_pts.append([X, Y, z_m])
            img_pts.append([u1, v1])

        if len(obj_pts) < MIN_PNP_POINTS:
            self.status = "DEPTH_FILTER"
            return

        obj_pts = np.asarray(obj_pts, dtype=np.float64)
        img_pts = np.asarray(img_pts, dtype=np.float64)

        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            obj_pts, img_pts, self.K, self.dist,
            flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=3.0,
            confidence=0.999, iterationsCount=150
        )
        
        if not ok or inl is None or len(inl) < 12:
            self.status = "PNP_FAIL"
            return

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3, 1)

        t_inv = (-R.T @ t).flatten()

        # INSTANT ABSOLUTE INTEGRATION
        step_n, step_e, step_d = vo_step_to_ned(
            float(t_inv[0]), float(t_inv[1]), float(t_inv[2]), live_yaw_rad
        )

        self.global_north += step_n
        self.global_east += step_e
        self.global_down += step_d

        self.status = "TRACKING"

        if (self.frame_idx % REDETECT_EVERY) == 0:
            pts = self._detect(gray)
            self.prev_pts = pts if pts is not None and len(pts) >= MIN_PNP_POINTS else curr_good.reshape(-1, 1, 2).astype(np.float32)
        else:
            self.prev_pts = curr_good.reshape(-1, 1, 2).astype(np.float32)

        self.prev_gray = gray.copy()
        self.prev_depth = depth_mm.copy()

    def apply_soft_correction(self, target_pose_xyz: np.ndarray):
        now = time.time()
        if now - self._last_corr_wall < SOFT_CORR_COOLDOWN: return None

        cur = np.array([self.global_north, self.global_east, self.global_down])
        drift = target_pose_xyz.reshape(3) - cur
        drift_mag = float(np.linalg.norm(drift))

        if drift_mag < MIN_DRIFT_TO_CORRECT_M: return None

        step = clamp_norm(drift, MAX_CORR_STEP_M)
        corr = step * SOFT_CORR_ALPHA

        # Apply soft correction directly to absolute tallies
        self.global_north += corr[0]
        self.global_east += corr[1]
        self.global_down += corr[2]
        
        self._last_corr_wall = now
        return True

    def pose(self):
        return [self.global_north, self.global_east, self.global_down]

# -----------------------
# Main Process Function
# -----------------------
def positioning(camera_frame_mutex, camera_calibration_mutex, attitude_mutex, position_mutex):
    W, H = 640, 400
    time.sleep(4) 

    # --- 1. MEMORY SETUP ---
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_calib = shared_memory.SharedMemory(name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(name="attitude")
    shm_position = shared_memory.SharedMemory(name="position")

    shared_calib = np.ndarray((3, 3), dtype=np.float64, buffer=shm_calib.buf)
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)

    local_calib = np.zeros((3, 3), dtype=np.float64)
    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_gray = np.zeros((H, W), dtype=np.uint8)
    local_depth = np.zeros((H, W), dtype=np.uint16)
    
    last_processed_gray = np.zeros((H, W), dtype=np.uint8)

    print("[VIO] Connected to Shared Memory. Booting Algorithm...")

    with camera_calibration_mutex:
        np.copyto(local_calib, shared_calib)
    
    vo = VO_LK(K=local_calib.copy())
    loop = LoopClosureORB()
    
    t0 = time.time()
    print("[VIO] Algorithm running. Calculating poses...\n")

    while True:
        # --- GET FRESHEST YAW ---
        with attitude_mutex:
            live_yaw_rad = shared_attitude[2] # Index 2 is Yaw

        # --- GET FRESHEST IMAGES ---
        with camera_frame_mutex:
            np.copyto(local_rgb, shared_rgb)
            np.copyto(local_gray, shared_gray)
            np.copyto(local_depth, shared_depth)

        if np.array_equal(local_gray, last_processed_gray):
            time.sleep(0.005) 
            continue
            
        np.copyto(last_processed_gray, local_gray)

        # --- HEAVY MATH ---
        t_sec = time.time() - t0
        vo.process(local_gray, local_depth, live_yaw_rad)

        # --- SLAM TOGGLE LOGIC ---
        # Set this to False dynamically when terminal landing sequence begins
        dynamic_slam_enabled = True 

        if dynamic_slam_enabled and vo.status == "TRACKING":
            pos_array = np.array(vo.pose())
            
            # Save keyframes
            if (vo.frame_idx % KEYFRAME_INTERVAL) == 0:
                loop.add_keyframe(local_rgb, pos_array, vo.frame_idx, t_sec)

            # Check for map loops and correct drift
            info = loop.check_loop(local_rgb, pos_array, vo.frame_idx)
            if info is not None:
                vo.apply_soft_correction(info["matched_pose"])

        # --- PUBLISH POSITION ---
        if vo.status == "TRACKING":
            pos = vo.pose()
            with position_mutex:
                shared_position[:] = pos
        else:
            print(f"VIO LOST. Status: {vo.status}")

# -----------------------
# Testing & Printing Process
# -----------------------
def test_positioning(position_mutex):
    # Wait 6 seconds to let the camera boot, memory allocate, and VIO math stabilize
    time.sleep(6) 
    
    # 1. Connect ONLY to the position shared memory
    shm_position = shared_memory.SharedMemory(name="position")
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)
    local_position = np.zeros((3,), dtype=np.float64)

    print("\n[PRINTER] Connected to Shared Memory. Listening for NED coordinates...\n")

    while True:
        # 2. Safely grab the latest coordinates
        with position_mutex:
            np.copyto(local_position, shared_position)

        # 3. Print them cleanly to the terminal
        print(f"[MISSION CONTROL SIM] Current Position -> North: {local_position[0]:+.2f}m | East: {local_position[1]:+.2f}m | Down: {local_position[2]:+.2f}m")
        
        # Only print twice a second so we don't spam the terminal
        time.sleep(0.5) 

if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2 
    CALIB_BYTES = 3 * 3 * 8 
    ATTITUDE_BYTES = 3 * 8 
    POSITION_BYTES = 3 * 8 # 3 float64 numbers (X, Y, Z = 24 bytes)

    # 1. Allocate all 6 shared memory blocks
    print("Positioning tester allocating shared memory...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(create=True, size=ATTITUDE_BYTES, name="attitude")
    shm_position = shared_memory.SharedMemory(create=True, size=POSITION_BYTES, name="position")
    
    # 2. Initialize all the locks
    camera_frame_mutex = mp.Lock()
    camera_calibration_mutex = mp.Lock()
    attitude_mutex = mp.Lock()
    position_mutex = mp.Lock()

    # 3. Define the three independent processes
    broadcaster_process = mp.Process(target=broadcaster, args=(camera_frame_mutex, camera_calibration_mutex, attitude_mutex))
    vio_process = mp.Process(target=positioning, args=(camera_frame_mutex, camera_calibration_mutex, attitude_mutex, position_mutex))
    printer_process = mp.Process(target=test_positioning, args=(position_mutex,))

    try:
        # 4. Start the stack
        broadcaster_process.start()
        vio_process.start()
        printer_process.start()

        # Keep the main process alive while the printer runs
        printer_process.join()
        
    except KeyboardInterrupt:
        print("\nPositioning tester caught keyboard interrupt. Shutting down...")
    finally:
        # 5. Terminate all child processes safely
        broadcaster_process.terminate()
        vio_process.terminate()
        printer_process.terminate()
        
        broadcaster_process.join()
        vio_process.join()
        printer_process.join()

        # 6. Clean up the shared memory to prevent leaks
        print("Positioning tester cleaning up shared memory...")
        shm_rgb.close()
        shm_rgb.unlink()
        shm_gray.close()
        shm_gray.unlink()
        shm_depth.close()
        shm_depth.unlink()
        shm_calib.close()
        shm_calib.unlink()
        shm_attitude.close()
        shm_attitude.unlink()
        shm_position.close()
        shm_position.unlink()
        print("Positioning tester processes terminated safely.")