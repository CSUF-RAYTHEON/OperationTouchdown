import cv2
import numpy as np
import time
import math
import multiprocessing as mp
from controls.busywait import delay_busywait
from controls.connect import connect_UART3
from vioslam.broadcaster import broadcaster 
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

MIN_PNP_POINTS = 9
DEPTH_MIN_M = 0.08
DEPTH_MAX_M = 18.0
REDETECT_EVERY = 10

# Loop closure (SLAM)
# --- SPATIAL KEYFRAMING SETTINGS ---
KEYFRAME_MIN_DIST_M = 0.08      # Saves a new map image if drone moves more than 8cm
KEYFRAME_MIN_YAW_RAD = 0.17   # Saves a new map image if drone rotates more than 0.30 radians

LOOP_CHECK_INTERVAL = 0.6 # Checks for a loop closure every 0.6 seconds
MIN_LOOP_SEPARATION = 15 # does not compare the live video against the 15 most recent images it just saved.
MATCH_THRESHOLD = 45
MAX_KEYFRAMES = 600 # Remembers 600 unique spatial locations
MAX_MATCH_CANDIDATES = 325 # Checks the last 325 frames for a match
ORB_NFEATURES = 400
ORB_SCALE = 0.5

# Soft drift correction
SOFT_CORR_ALPHA = 0.25 # Instead of applying the full correction, it multiplies the distance by SOFT_CORR_ALPHA. It only nudges the VIO coordinates 25% closer to the truth.
SOFT_CORR_COOLDOWN = 0.75 # Time before another soft correction can be applied, in seconds
MIN_DRIFT_TO_CORRECT_M = 0.12 # minimum drift required to apply a correction in meters (if the drift is smaller than this, we just let it be to avoid over-correcting and adding noise)
MAX_CORR_STEP_M = 1.0 # caps the maximum correction distance to 1.0 meter per frame

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

def vo_step_to_ned(cam_x, cam_y, cam_z, roll_rad, pitch_rad, yaw_rad):
    """
    Takes the relative camera movement step and rotates it through the 
    drone's full 3D attitude (Roll, Pitch, Yaw) to calculate the 
    true absolute step in the global magnetic NED frame.
    """
    # 1. Map Camera Optical Frame to Drone Body Frame (Downward Camera)
    # Moving towards top of image = Forward (North)
    # Moving towards right of image = Right (East)
    # Moving straight out of lens = Down
    body_x = -cam_y  
    body_y = cam_x   
    body_z = cam_z   

    # 2. Generate Euler Z-Y-X Rotation Matrix (Body to NED)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)

    # Matrix multiplication pre-calculated for extreme execution speed
    R00 = cy * cp
    R01 = cy * sp * sr - sy * cr
    R02 = cy * sp * cr + sy * sr

    R10 = sy * cp
    R11 = sy * sp * sr + cy * cr
    R12 = sy * sp * cr - cy * sr

    R20 = -sp
    R21 = cp * sr
    R22 = cp * cr

    # 3. Apply the 3D rotation to the step vector
    step_n = R00 * body_x + R01 * body_y + R02 * body_z
    step_e = R10 * body_x + R11 * body_y + R12 * body_z
    step_d = R20 * body_x + R21 * body_y + R22 * body_z

    return step_n, step_e, step_d

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
        if frame is None: return None
        if len(frame.shape) == 2: return frame
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def _prep(self, frame: np.ndarray) -> np.ndarray:
        gray = self._to_gray(frame)
        if gray is None: return None
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

    def _reset_tracking(self, gray, depth_mm):
        self.prev_gray = gray.copy()
        self.prev_depth = depth_mm.copy()
        self.prev_pts = self._detect(gray)

    def process(self, gray, depth_mm, roll_rad, pitch_rad, yaw_rad):
        self.frame_idx += 1
        W, H = gray.shape[1], gray.shape[0]

        if self.prev_gray is None or self.prev_depth is None or self.prev_pts is None:
            self._reset_tracking(gray, depth_mm)
            self.status = "WARMUP"
            return

        next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=LK_WIN_SIZE, maxLevel=LK_MAX_LEVEL,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        if next_pts is None or st is None:
            self.status = "LK_FAIL"
            self._reset_tracking(gray, depth_mm) 
            return

        st = st.reshape(-1)
        prev_good = self.prev_pts[st == 1].reshape(-1, 2)
        curr_good = next_pts[st == 1].reshape(-1, 2)
        self.num_tracked = len(prev_good)

        if self.num_tracked < MIN_PNP_POINTS:
            self.status = f"LOW_TRACK({self.num_tracked})"
            self._reset_tracking(gray, depth_mm) 
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
            self._reset_tracking(gray, depth_mm) 
            return

        obj_pts = np.asarray(obj_pts, dtype=np.float64)
        img_pts = np.asarray(img_pts, dtype=np.float64)

        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            obj_pts, img_pts, self.K, self.dist,
            flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=3.0,
            confidence=0.999, iterationsCount=150
        )
        
        if not ok or inl is None or len(inl) < 9:
            self.status = "PNP_FAIL"
            self._reset_tracking(gray, depth_mm) 
            return

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3, 1)

        t_inv = (-R.T @ t).flatten()

        # 3D ABSOLUTE INTEGRATION
        step_n, step_e, step_d = vo_step_to_ned(
            float(t_inv[0]), float(t_inv[1]), float(t_inv[2]), 
            roll_rad, pitch_rad, yaw_rad
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
def positioning(camera_frame_mutex, camera_calibration_mutex, attitude_mutex):
    master_uart3 = connect_UART3()
    W, H = 640, 400
    # --- 1. MEMORY SETUP ---
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_calib = shared_memory.SharedMemory(name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(name="attitude")

    shared_calib = np.ndarray((3, 3), dtype=np.float64, buffer=shm_calib.buf)
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)

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
    
    # --- SPATIAL KEYFRAMING TRACKERS ---
    last_kf_pos = None
    last_kf_yaw = None
    
    print("[VIO] Algorithm running. Calculating poses...\n")

    while True:
        # --- GET FRESHEST ATTITUDE ---
        with attitude_mutex:
            live_roll = shared_attitude[0]
            live_pitch = shared_attitude[1]
            live_yaw = shared_attitude[2]

        # --- GET FRESHEST IMAGES ---
        with camera_frame_mutex:
            np.copyto(local_rgb, shared_rgb)
            np.copyto(local_gray, shared_gray)
            np.copyto(local_depth, shared_depth)

        if np.array_equal(local_gray, last_processed_gray):
            delay_busywait(0.001) 
            continue
        timestamp_usec = int(time.time() * 1e6)
        np.copyto(last_processed_gray, local_gray)

        # --- HEAVY MATH ---
        t_sec = time.time() - t0
        vo.process(local_gray, local_depth, live_roll, live_pitch, live_yaw)

        # --- SLAM TOGGLE LOGIC ---
        dynamic_slam_enabled = True 

        if dynamic_slam_enabled and vo.status == "TRACKING":
            pos_array = np.array(vo.pose())
            
            # --- THE NEW SPATIAL CHECK ---
            save_kf = False
            
            # If we don't have a baseline yet, save the very first frame immediately
            if last_kf_pos is None:
                save_kf = True
            else:
                # Calculate the 3D physical distance we traveled since the last picture
                dist_moved = float(np.linalg.norm(pos_array - last_kf_pos))
                
                # Calculate how far the drone rotated (in radians) since the last picture
                yaw_changed = abs(wrap_rad_pi(live_yaw - last_kf_yaw))
                
                # If we moved far enough, OR rotated far enough, trigger a save
                if dist_moved >= KEYFRAME_MIN_DIST_M or yaw_changed >= KEYFRAME_MIN_YAW_RAD:
                    save_kf = True

            if save_kf:
                loop.add_keyframe(local_rgb, pos_array, vo.frame_idx, t_sec)
                # Update our baseline to the exact spot we just saved
                last_kf_pos = pos_array.copy()
                last_kf_yaw = live_yaw

            # 2. Check for map loops (Still runs every 0.6 seconds based on wall-clock)
            info = loop.check_loop(local_rgb, pos_array, vo.frame_idx)
            if info is not None:
                vo.apply_soft_correction(info["matched_pose"])
                
                # Optional: Overwrite our last_kf_pos with the newly corrected coordinates 
                # so the teleport doesn't instantly trigger a false spatial keyframe
                last_kf_pos = np.array(vo.pose())

        # --- PUBLISH POSITION ---
        if vo.status == "TRACKING":
            pos = vo.pose() # pos[0] = North (X) pos[1] = East (Y) pos[2] = Down (Z)
            master_uart3.mav.vision_position_estimate_send(timestamp_usec, pos[0], pos[1], pos[2], 0.0, 0.0, 0.0)
        else:
            print(f"VIO LOST. Status: {vo.status}")
# -----------------------
# Testing & Printing Process
# -----------------------
def positioning_test(camera_frame_mutex, camera_calibration_mutex, attitude_mutex, position_mutex, slam_enabled_mutex):
    W, H = 640, 400
    # --- 1. MEMORY SETUP ---
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_calib = shared_memory.SharedMemory(name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(name="attitude")
    shm_position = shared_memory.SharedMemory(name="position")
    shm_slam_enabled = shared_memory.SharedMemory(name="slam_enabled")

    shared_calib = np.ndarray((3, 3), dtype=np.float64, buffer=shm_calib.buf)
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)
    shared_slam_enabled = np.ndarray((1,), dtype=np.bool_, buffer=shm_slam_enabled.buf)

    local_calib = np.zeros((3, 3), dtype=np.float64)
    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_gray = np.zeros((H, W), dtype=np.uint8)
    local_depth = np.zeros((H, W), dtype=np.uint16)
    
    last_processed_gray = np.zeros((H, W), dtype=np.uint8)
    vo_average_time = 0.0
    vo_count = 0
    slam_count = 0
    slam_average_time = 0.0
    slam_enabled = True
    with slam_enabled_mutex:
        shared_slam_enabled[0] = slam_enabled

    print("[VIO] Connected to Shared Memory. Booting Algorithm...")

    with camera_calibration_mutex:
        np.copyto(local_calib, shared_calib)
    
    vo = VO_LK(K=local_calib.copy())
    loop = LoopClosureORB()
    
    t0 = time.time()
    
    # --- SPATIAL KEYFRAMING TRACKERS ---
    last_kf_pos = None
    last_kf_yaw = None
    
    print("[VIO] Algorithm running. Calculating poses...\n")

    while True:
        start_time = time.perf_counter()
        # --- GET FRESHEST ATTITUDE ---
        with attitude_mutex:
            live_roll = shared_attitude[0]
            live_pitch = shared_attitude[1]
            live_yaw = shared_attitude[2]

        # --- GET FRESHEST IMAGES ---
        with camera_frame_mutex:
            np.copyto(local_rgb, shared_rgb)
            np.copyto(local_gray, shared_gray)
            np.copyto(local_depth, shared_depth)

        if np.array_equal(local_gray, last_processed_gray):
            delay_busywait(0.001)
            continue
            
        np.copyto(last_processed_gray, local_gray)

        # --- HEAVY MATH ---
        t_sec = time.time() - t0
        vo.process(local_gray, local_depth, live_roll, live_pitch, live_yaw)
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000.0
        vo_count += 1
        vo_average_time += elapsed_ms
        if vo_count % 256 == 0:
            vo_average_time /= vo_count
            vo_count = 0
            print(f"[VIO] Average processing time per frame: {vo_average_time:.2f} ms")
            vo_average_time = 0.0

        # --- SLAM TOGGLE LOGIC ---
        with slam_enabled_mutex:
            slam_enabled = bool(shared_slam_enabled[0])

        if slam_enabled and vo.status == "TRACKING":
            pos_array = np.array(vo.pose())
            
            # --- THE NEW SPATIAL CHECK ---
            save_kf = False
            
            # If we don't have a baseline yet, save the very first frame immediately
            if last_kf_pos is None:
                save_kf = True
            else:
                # Calculate the 3D physical distance we traveled since the last picture
                dist_moved = float(np.linalg.norm(pos_array - last_kf_pos))
                
                # Calculate how far the drone rotated (in radians) since the last picture
                yaw_changed = abs(wrap_rad_pi(live_yaw - last_kf_yaw))
                
                # If we moved far enough, OR rotated far enough, trigger a save
                if dist_moved >= KEYFRAME_MIN_DIST_M or yaw_changed >= KEYFRAME_MIN_YAW_RAD:
                    save_kf = True

            if save_kf:
                loop.add_keyframe(local_rgb, pos_array, vo.frame_idx, t_sec)
                # Update our baseline to the exact spot we just saved
                last_kf_pos = pos_array.copy()
                last_kf_yaw = live_yaw

            # 2. Check for map loops (Still runs every 0.6 seconds based on wall-clock)
            start_time = time.perf_counter()
            info = loop.check_loop(local_rgb, pos_array, vo.frame_idx)
            if info is not None:
                vo.apply_soft_correction(info["matched_pose"])
                
                # Optional: Overwrite our last_kf_pos with the newly corrected coordinates 
                # so the teleport doesn't instantly trigger a false spatial keyframe
                last_kf_pos = np.array(vo.pose())
            end_time = time.perf_counter()
            elapsed_ms = (end_time - start_time) * 1000.0
            if elapsed_ms > 12.0:
                slam_count += 1
                slam_average_time += elapsed_ms
            if slam_count % 16 == 0:
                slam_average_time /= slam_count
                slam_count = 0
                print(f"[SLAM] Average loop check time: {slam_average_time:.2f} ms")
                slam_average_time = 0.0
        # --- PUBLISH POSITION ---
        if vo.status == "TRACKING":
            pos = vo.pose()
            with position_mutex:
                shared_position[:] = pos
        else:
            print(f"VIO LOST. Status: {vo.status}")

def test_positioning(position_mutex, slam_enabled_mutex):    
    shm_position = shared_memory.SharedMemory(name="position")
    shm_slam_enabled = shared_memory.SharedMemory(name="slam_enabled")
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)
    shared_slam_enabled = np.ndarray((1,), dtype=bool, buffer=shm_slam_enabled.buf)
    local_position = np.zeros((3,), dtype=np.float64)


    print("\n[PRINTER] Connected to Shared Memory. Listening for NED coordinates...\n")

    while True:
        with position_mutex:
            np.copyto(local_position, shared_position)
        print(f"[MISSION CONTROL SIM] Current Position -> North: {local_position[0]:+.2f}m | East: {local_position[1]:+.2f}m | Down: {local_position[2]:+.2f}m")

        if local_position[0] >= 9:
            print("\n[MISSION CONTROL SIM] Target reached! Disabling SLAM corrections...\n")
            with slam_enabled_mutex:
                shared_slam_enabled[0] = False
        time.sleep(0.1) 

if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2 
    CALIB_BYTES = 3 * 3 * 8 
    ATTITUDE_BYTES = 3 * 8 
    POSITION_BYTES = 3 * 8 
    LOCAL_POSITION_NED_BYTES = 3 * 8
    BOOL_BYTES = 1

    print("Positioning tester allocating shared memory...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(create=True, size=ATTITUDE_BYTES, name="attitude")
    shm_position = shared_memory.SharedMemory(create=True, size=POSITION_BYTES, name="position")
    shm_local_position_ned = shared_memory.SharedMemory(create=True, size=LOCAL_POSITION_NED_BYTES, name="local_position_ned")
    shm_slam_enabled = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_enabled")
    
    camera_frame_mutex = mp.Lock()
    camera_calibration_mutex = mp.Lock()
    attitude_mutex = mp.Lock()
    position_mutex = mp.Lock()
    local_position_ned_mutex = mp.Lock()
    slam_enabled_mutex = mp.Lock()

    broadcaster_process = mp.Process(target=broadcaster, args=(camera_frame_mutex, camera_calibration_mutex, attitude_mutex, local_position_ned_mutex))
    vio_process = mp.Process(target=positioning_test, args=(camera_frame_mutex, camera_calibration_mutex, attitude_mutex, position_mutex, slam_enabled_mutex))
    test_process = mp.Process(target=test_positioning, args=(position_mutex, slam_enabled_mutex))

    try:
        broadcaster_process.start()
        time.sleep(3)
        vio_process.start()
        time.sleep(3)
        test_process.start()
        time.sleep(3)
        test_process.join()
        
    except KeyboardInterrupt:
        print("\nPositioning tester caught keyboard interrupt. Shutting down...")
    finally:
        broadcaster_process.terminate()
        vio_process.terminate()
        test_process.terminate()
        
        broadcaster_process.join()
        vio_process.join()
        test_process.join()

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
        shm_local_position_ned.close()
        shm_local_position_ned.unlink()
        print("Positioning tester processes terminated safely.")