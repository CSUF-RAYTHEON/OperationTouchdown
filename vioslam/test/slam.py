import cv2
import numpy as np
import time
import math
import multiprocessing as mp
from controls.busywait import delay_busywait
from vioslam.broadcaster import broadcaster 
from multiprocessing import shared_memory
from pymavlink import mavutil

KEYFRAME_MIN_DIST_M = 0.35 # Must move at least 35cm to save a map image. Increasing saves RAM and CPU by creating a sparser map, but risks missing loop closures. Decreasing creates a dense, highly accurate map but fills memory rapidly but more prone to self-matching 
KEYFRAME_MIN_DIST_RATIO = 0.25 # Saves a map image if the drone moves 25% relative to its current altitude. Decreasing makes a denser map but risks self-matching, increasing makes a sparser map and saves resources and lowers the risk of self-matching bug
KEYFRAME_MIN_YAW_RAD = 1.0 # Saves a map image if drone rotates 1 radian. Increasing requires sharp turns to trigger a save. Decreasing maps curves better but eats memory if the drone just wobbles.
LOOP_CHECK_INTERVAL = 0.5 # Seconds between map searches. Increasing saves CPU by checking less often, but lets drift accumulate longer. Decreasing fixes drift instantly but constantly hammers the CPU with heavy math.
MIN_LOOP_SEPARATION = 30 # Ignores the x most recent frames. Increasing strictly prevents the drone from matching with where it was however many seconds ago. Decreasing causes wasted CPU cycles comparing the live feed against the immediate past.
MATCH_THRESHOLD = 50 # Minimum perfect ORB matches required to trigger a correction. Increasing guarantees zero false teleports but makes the system overly strict. Decreasing finds loops easily but risks a catastrophic crash if it falsely matches two similar-looking floor tiles.
MAX_KEYFRAMES = 600 # Max total images held in RAM. Increasing lets the drone remember massive flight paths (e.g., a whole building). Decreasing saves RAM but causes the drone to "forget" its takeoff point on long flights.
MAX_MATCH_CANDIDATES = 325 # Max images searched per cycle. Increasing finds loops deeper in history but makes SLAM math take much longer (e.g., 400ms+). Decreasing keeps the SLAM delay short but blinds the algorithm to older map areas.
ORB_NFEATURES = 400 # Visual tracking points per image. Increasing creates incredibly robust map matches but quadratically explodes the Brute Force CPU math. Decreasing makes SLAM lightning fast but risks failing to find matches on smooth or blurry floors.
ORB_SCALE = 0.5 # Shrinks the image to 50% before processing. Increasing (to 1.0) gets razor-sharp tracking features but slows down detection. Decreasing (e.g., 0.25) makes feature extraction instant but the pixel data becomes too blocky to reliably match.
MIN_ALTITUTDE_CORRECTION_INTERVAL = 3.0 # Seconds between altitude corrections. Increasing lets altitude drift accumulate longer but saves CPU by checking less often. Decreasing fixes altitude drift more frequently but  hammers the CPU with heavy math.

def wrap_rad_pi(angle_rad: float) -> float:
    while angle_rad > math.pi: angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi: angle_rad += 2.0 * math.pi
    return angle_rad
class LoopClosureORB:
    def __init__(self):
        self.orb = cv2.ORB_create(nfeatures=ORB_NFEATURES)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.keyframes = []
        self.last_check_wall = 0.0
        self.last_valid_kf_index = 0
        self.last_altitude_correction_wall = 0.0

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
            if self.last_valid_kf_index > 0:
            # Shift the checkpoint left to match the array
                self.last_valid_kf_index -= 1

    def check_loop(self, frame: np.ndarray, current_pose_xyz: np.ndarray, frame_id: int):
        if time.time() - self.last_check_wall < LOOP_CHECK_INTERVAL: return None

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
        # No match was found
        if best is None or best_score < MATCH_THRESHOLD:
            # set the last valid kf index to the most recent keyframe
            self.last_valid_kf_index = len(self.keyframes) - 1
            self.last_check_wall = time.time()
            return None
        # Match found
        self.cull_keyframes()
        drift_vec = best["pose"] - current_pose_xyz
        drift_mag = float(np.linalg.norm(drift_vec))
        self.last_check_wall = time.time()
        return (drift_mag, float(drift_vec[0]), float(drift_vec[1]), float(drift_vec[2]))
    
    def cull_keyframes(self):
            if len(self.keyframes) - 1 > self.last_valid_kf_index:
                del self.keyframes[self.last_valid_kf_index + 1:]
    
    def altitude_correction(self, live_attitude, local_position, depth_frame_mutex, slam_trigger_mutex, shared_depth, shared_slam_trigger, shared_slam_target):
        # 1. Limit altitude corrections to at most once every MIN_ALTITUTDE_CORRECTION_INTERVAL seconds
        if time.time() - self.last_altitude_correction_wall < MIN_ALTITUTDE_CORRECTION_INTERVAL: return None
        # Avoid getting depth frame until MIN_ALTITUTDE_CORRECTION_INTERVAL has passed to avoid mutex contention with the VIO process which also needs the depth frame
        with depth_frame_mutex:
            region_of_interest = shared_depth[180:220, 300:340].copy() 
            
        # 2. Filter out stereo blind spots (zeros)
        valid_depths = region_of_interest[region_of_interest > 0] 
        
        if len(valid_depths) > 0:
            # 3. Calculate median and convert to meters
            median_mm = np.median(valid_depths)
            depth_m = median_mm / 1000.0
            # 4. Calculate true vertical height and convert to NED (negative Z)
            true_alt_m = depth_m * math.cos(live_attitude[1]) * math.cos(live_attitude[0])
            true_z_ned = -true_alt_m 
            # 5. Calculate how far VIO has drifted from reality
            z_drift = true_z_ned - local_position[2]
            
            # 6. Only correct if VIO has drifted more than 5cm
            if abs(z_drift) > 0.05:
                with slam_trigger_mutex:
                    # If the slam is already being corrected by a loop closure, no need to do an attitude correction
                    if not shared_slam_trigger[0]:
                    # Send [Magnitude, X, Y, Z]. Zeroing X and Y protects the 2D map.
                        shared_slam_target[:] = [abs(z_drift), 0.0, 0.0, z_drift]
                        shared_slam_trigger[0] = True        
        # Update the timer whether it triggered a correction or not
        self.last_altitude_correction_wall = time.time()
def slam(rgb_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_enabled_mutex, slam_trigger_mutex):
    W, H = 640, 400
    
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_attitude = shared_memory.SharedMemory(name="attitude")
    shm_position = shared_memory.SharedMemory(name="position")
    shm_slam_target = shared_memory.SharedMemory(name="slam_target")
    shm_slam_trigger = shared_memory.SharedMemory(name="slam_trigger")
    shm_slam_enabled = shared_memory.SharedMemory(name="slam_enabled")

    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)
    shared_slam_target = np.ndarray((4,), dtype=np.float64, buffer=shm_slam_target.buf)
    shared_slam_trigger = np.ndarray((1,), dtype=np.bool_, buffer=shm_slam_trigger.buf)
    shared_slam_enabled = np.ndarray((1,), dtype=np.bool_, buffer=shm_slam_enabled.buf)

    last_processed_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_position = np.zeros((3,), dtype=np.float64)

    loop = LoopClosureORB()
    t0 = time.time()
    
    last_kf_pos = None
    last_kf_yaw = None
    slam_frame_id = 0
    with slam_enabled_mutex:
        shared_slam_enabled[0] = True
    print("SLAM setup complete and running")

    while True:
        with slam_enabled_mutex:
            slam_enabled = bool(shared_slam_enabled[0])
        if not slam_enabled:
            time.sleep(2.5)
            continue

        with rgb_frame_mutex:
            np.copyto(local_rgb, shared_rgb)
        if np.array_equal(local_rgb, last_processed_rgb):
            delay_busywait(0.005)
            continue
        
        np.copyto(last_processed_rgb, local_rgb)
        slam_frame_id += 1
        t_sec = time.time() - t0

        with attitude_mutex:
            live_attitude = shared_attitude[:]
        with position_mutex:
            np.copyto(local_position, shared_position)

        if last_kf_pos is None or last_kf_yaw is None:
            loop.add_keyframe(local_rgb, local_position, slam_frame_id, t_sec)
            last_kf_pos = local_position.copy()
            last_kf_yaw = live_attitude[2]
        else:
            yaw_changed = abs(wrap_rad_pi(live_attitude[2] - last_kf_yaw))
            if yaw_changed >= KEYFRAME_MIN_YAW_RAD:
                loop.add_keyframe(local_rgb, local_position, slam_frame_id, t_sec)
                last_kf_pos = local_position.copy()
                last_kf_yaw = live_attitude[2]
            else:
                dist_moved = float(np.linalg.norm(local_position - last_kf_pos))
                dynamic_min_dist = max(KEYFRAME_MIN_DIST_M, abs(last_kf_pos[2]) * KEYFRAME_MIN_DIST_RATIO)
                if dist_moved >= dynamic_min_dist:
                    loop.add_keyframe(local_rgb, local_position, slam_frame_id, t_sec)
                    last_kf_pos = local_position.copy()
                    last_kf_yaw = live_attitude[2]
        # 2. LOOP CLOSURE SEARCH
        info = loop.check_loop(local_rgb, local_position, slam_frame_id)
        if info is not None:
            with slam_trigger_mutex:
                shared_slam_target[:] = info
                shared_slam_trigger[0] = True
            # Overwrite baseline so teleport doesn't instantly trigger a false spatial keyframe
            last_kf_pos = info["matched_pose"].copy()
        else:
            loop.altitude_correction(live_attitude, local_position, depth_frame_mutex, slam_trigger_mutex, shared_depth, shared_slam_trigger, shared_slam_target)
def test_latency_slam(rgb_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_enabled_mutex, slam_trigger_mutex):
    W, H = 640, 400
    count = 0
    average_ms = 0.0
    
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_attitude = shared_memory.SharedMemory(name="attitude")
    shm_position = shared_memory.SharedMemory(name="position")
    shm_slam_target = shared_memory.SharedMemory(name="slam_target")
    shm_slam_trigger = shared_memory.SharedMemory(name="slam_trigger")
    shm_slam_enabled = shared_memory.SharedMemory(name="slam_enabled")

    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)
    shared_slam_target = np.ndarray((4,), dtype=np.float64, buffer=shm_slam_target.buf)
    shared_slam_trigger = np.ndarray((1,), dtype=np.bool_, buffer=shm_slam_trigger.buf)
    shared_slam_enabled = np.ndarray((1,), dtype=np.bool_, buffer=shm_slam_enabled.buf)

    last_processed_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_position = np.zeros((3,), dtype=np.float64)

    loop = LoopClosureORB()
    t0 = time.time()
    
    last_kf_pos = None
    last_kf_yaw = None
    slam_frame_id = 0
    with slam_enabled_mutex:
        shared_slam_enabled[0] = True
    print("SLAM Latency tester setup complete and running")

    while True:
        start_time = time.perf_counter()
        with slam_enabled_mutex:
            slam_enabled = bool(shared_slam_enabled[0])
        if not slam_enabled:
            time.sleep(2.5)
            continue

        with rgb_frame_mutex:
            np.copyto(local_rgb, shared_rgb)
        if np.array_equal(local_rgb, last_processed_rgb):
            delay_busywait(0.005)
            continue
        
        np.copyto(last_processed_rgb, local_rgb)
        slam_frame_id += 1
        t_sec = time.time() - t0

        with attitude_mutex:
            live_attitude = shared_attitude[:]
        with position_mutex:
            np.copyto(local_position, shared_position)

        if last_kf_pos is None or last_kf_yaw is None:
            loop.add_keyframe(local_rgb, local_position, slam_frame_id, t_sec)
            last_kf_pos = local_position.copy()
            last_kf_yaw = live_attitude[2]
        else:
            yaw_changed = abs(wrap_rad_pi(live_attitude[2] - last_kf_yaw))
            if yaw_changed >= KEYFRAME_MIN_YAW_RAD:
                loop.add_keyframe(local_rgb, local_position, slam_frame_id, t_sec)
                last_kf_pos = local_position.copy()
                last_kf_yaw = live_attitude[2]
            else:
                dist_moved = float(np.linalg.norm(local_position - last_kf_pos))
                dynamic_min_dist = max(KEYFRAME_MIN_DIST_M, abs(last_kf_pos[2]) * KEYFRAME_MIN_DIST_RATIO)
                if dist_moved >= dynamic_min_dist:
                    loop.add_keyframe(local_rgb, local_position, slam_frame_id, t_sec)
                    last_kf_pos = local_position.copy()
                    last_kf_yaw = live_attitude[2]
        # 2. LOOP CLOSURE SEARCH
        info = loop.check_loop(local_rgb, local_position, slam_frame_id)
        if info is not None:
            with slam_trigger_mutex:
                shared_slam_target[:] = info
                shared_slam_trigger[0] = True
            # Update the baseline by adding the drift vector
            last_kf_pos += np.array([info[1], info[2], info[3]])
            end_time = time.perf_counter()
            time_elapsed_ms = (end_time - start_time) * 1000.0
            if time_elapsed_ms > 5.0:
                average_ms += time_elapsed_ms
                count += 1
            if count >= 8:
                print(f"Average SLAM: {average_ms / count:.3f} ms, Map size: {len(loop.keyframes)}")
                count = 0
                average_ms = 0.0
        else:
            loop.altitude_correction(live_attitude, local_position, depth_frame_mutex, slam_trigger_mutex, shared_depth, shared_slam_trigger, shared_slam_target)
def main(position_mutex):
    shm_position = shared_memory.SharedMemory(name="position")
    shared_position = np.ndarray((3,), dtype=np.float64, buffer=shm_position.buf)
    local_position = np.zeros((3,), dtype=np.float64)

    print("Main Connected to Shared Memory. Listening for NED coordinates...")
    while True:
        with position_mutex:
            np.copyto(local_position, shared_position)

        print(f"North:{local_position[0]:+.2f}m | East: {local_position[1]:+.2f}m | Down: {local_position[2]:+.2f}m")
        time.sleep(0.25) 
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
    TARGET_BYTES = 4 * 8

    print("SLAM tester allocating shared memory...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(create=True, size=ATTITUDE_BYTES, name="attitude")
    shm_position = shared_memory.SharedMemory(create=True, size=POSITION_BYTES, name="position")
    shm_local_position_ned = shared_memory.SharedMemory(create=True, size=LOCAL_POSITION_NED_BYTES, name="local_position_ned")
    shm_slam_enabled = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_enabled")
    shm_slam_target = shared_memory.SharedMemory(create=True, size=TARGET_BYTES, name="slam_target")
    shm_slam_trigger = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_trigger")
    print("SLAM tester finished allocating shared memory...")

    rgb_frame_mutex = mp.Lock()
    gray_frame_mutex = mp.Lock()
    depth_frame_mutex = mp.Lock()
    attitude_mutex = mp.Lock()
    position_mutex = mp.Lock()
    local_position_ned_mutex = mp.Lock()
    slam_trigger_mutex = mp.Lock()
    slam_enabled_mutex = mp.Lock()

    from vioslam.vio import vio
    broadcaster_process = mp.Process(target=broadcaster, args=(rgb_frame_mutex, gray_frame_mutex, depth_frame_mutex, attitude_mutex, local_position_ned_mutex,))
    vio_process = mp.Process(target=vio, args=(gray_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_trigger_mutex,))
    slam_process = mp.Process(target=test_latency_slam, args=(rgb_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_enabled_mutex, slam_trigger_mutex,))
    main_process = mp.Process(target=main, args=(position_mutex,))

    try:
        broadcaster_process.start()
        time.sleep(3)
        vio_process.start()
        time.sleep(3)
        slam_process.start()
        time.sleep(3)
        main_process.start()
        time.sleep(3)
        main_process.join()
        
    except KeyboardInterrupt:
        print("SLAM tester caught keyboard interrupt. Shutting down...")
    finally:
        broadcaster_process.terminate()
        vio_process.terminate()
        slam_process.terminate()
        main_process.terminate()
        
        broadcaster_process.join()
        vio_process.join()
        slam_process.join()
        main_process.join()

        print("SLAM tester cleaning up shared memory...")
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
        shm_slam_enabled.close()
        shm_slam_enabled.unlink()
        shm_slam_target.close()
        shm_slam_target.unlink()
        shm_slam_trigger.close()
        shm_slam_trigger.unlink()
        print("SLAM tester processes terminated safely.")