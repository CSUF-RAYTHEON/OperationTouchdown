import cv2
import numpy as np
import cv2.aruco as aruco

class NestedAprilTagDetector:
    def __init__(self, camera_matrix_3x3):
        # Physical tag sizes in meters
        self.OUTER_ID = 77      
        self.INNER_ID = 12      
        self.OUTER_SIZE = 0.34  
        self.INNER_SIZE = 0.09  

        # ArUco Setup
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(dictionary, params)
        
        # CLAHE for handling harsh shadows/sunlight splits
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # 3D Object Geometries (Tag corners centered at 0,0,0)
        def make_obj_pts(size):
            half = size / 2.0
            return np.array([
                [-half,  half, 0.0], [ half,  half, 0.0],
                [ half, -half, 0.0], [-half, -half, 0.0],
            ], dtype=np.float32)
            
        self.obj_inner = make_obj_pts(self.INNER_SIZE)
        self.obj_outer = make_obj_pts(self.OUTER_SIZE)
        
        # Intrinsics
        self.camera_matrix = np.array(camera_matrix_3x3, dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _extract_ned_pose(self, corners, obj_points, yaw_rad):
        """Converts 2D pixel corners to 3D NED coordinates."""
        success, rvec, tvec = cv2.solvePnP(
            obj_points, corners.astype(np.float32),
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success: 
            return None
        
        # 1. Translate Camera frame to Drone Body frame (Forward, Right, Down)
        # Assuming camera is mounted flat, pointing straight down
        cam_x, cam_y, cam_z = tvec[0][0], tvec[1][0], tvec[2][0]
        forward_m, right_m, down_m = -cam_y, cam_x, cam_z
        
        # 2. Rotate Body frame into Local NED using Drone's Yaw
        north_m = forward_m * np.cos(yaw_rad) - right_m * np.sin(yaw_rad)
        east_m = forward_m * np.sin(yaw_rad) + right_m * np.cos(yaw_rad)
        
        return north_m, east_m, down_m

    def get_target_ned(self, frame, yaw_rad=0.0):
        """
        Inputs: 
            frame: 2D Grayscale array (from OAK-D Mono Global Shutter)
            yaw_rad: Drone's current yaw in radians
        Outputs: 
            Dict containing relative North, East, Down distances in meters.
        """
        # Ensure frame is grayscale and apply mild contrast enhancement
        if frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(frame)

        # PASS 1: Detect tags normally
        corners1, ids1, _ = self.detector.detectMarkers(gray)
        found = {}
        if ids1 is not None:
            for i, tid in enumerate(ids1.flatten()):
                found[tid] = corners1[i].reshape(4, 2)

        # PRIORITY 1: Always use the Inner Tag if visible
        if self.INNER_ID in found:
            pose = self._extract_ned_pose(found[self.INNER_ID], self.obj_inner, yaw_rad)
            if pose: return {"layer": "inner", "north": pose[0], "east": pose[1], "down": pose[2]}

        # If we saw the outer tag perfectly on pass 1, use it
        if self.OUTER_ID in found:
            pose = self._extract_ned_pose(found[self.OUTER_ID], self.obj_outer, yaw_rad)
            if pose: return {"layer": "outer", "north": pose[0], "east": pose[1], "down": pose[2]}

        # PASS 2: Masking Recovery for Outer Tag
        # If we saw *something* but no valid target yet, try masking out the inner geometry
        if ids1 is not None:
            masked = gray.copy()
            for c in corners1:
                pts = c.reshape((-1, 2)).astype(np.float32)
                center = pts.mean(axis=0)

                # Create concentric rings to sample the white background and fill the tag
                ring_outer = (center + 1.35 * (pts - center)).astype(np.int32)
                ring_inner = (center + 1.10 * (pts - center)).astype(np.int32)
                ring_mask = np.zeros_like(gray)
                cv2.fillPoly(ring_mask, [ring_outer], 255)
                cv2.fillPoly(ring_mask, [ring_inner], 0)
                
                fill_val = int(np.median(gray[ring_mask == 255])) if np.any(ring_mask) else 255
                fill_poly = (center + 1.25 * (pts - center)).astype(np.int32)
                cv2.fillPoly(masked, [fill_poly], color=fill_val)

            # Re-run detection on the masked image
            corners2, ids2, _ = self.detector.detectMarkers(masked)
            if ids2 is not None:
                for i, tid in enumerate(ids2.flatten()):
                    if tid == self.OUTER_ID:
                        pose = self._extract_ned_pose(corners2[i].reshape(4, 2), self.obj_outer, yaw_rad)
                        if pose: return {"layer": "outer", "north": pose[0], "east": pose[1], "down": pose[2]}

        return None

def main(gray_frame_mutex, depth_frame_mutex, attitude_mutex):
    from controls.affinitypriority import set_core_and_priority
    from controls.busywait import delay_busywait
    import multiprocessing as mp
    from multiprocessing import shared_memory
    set_core_and_priority(3, None) # Core 3, Normal Priority
    W, H = 640, 400

    # 1. Connect to shared memory blocks
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_calib = shared_memory.SharedMemory(name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(name="attitude")

    # 2. Create NumPy arrays backed by shared memory
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_calib = np.ndarray((3, 3), dtype=np.float64, buffer=shm_calib.buf)
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)

    # 3. Create local copies to prevent holding the mutex during heavy processing
    local_calib = np.zeros((3, 3), dtype=np.float64)
    local_gray = np.zeros((H, W), dtype=np.uint8)
    local_attitude = np.zeros((3,), dtype=np.float64)
    last_processed_gray = np.zeros((H, W), dtype=np.uint8)

    # 4. Grab the camera matrix once (Broadcaster writes it using depth_frame_mutex)
    with depth_frame_mutex:
        np.copyto(local_calib, shared_calib)

    # 5. Initialize the tracker
    tracker = NestedAprilTagDetector(local_calib)
    print("Landing Target Detector setup complete and running")

    while True:
        # Fetch the latest gray frame
        with gray_frame_mutex:
            np.copyto(local_gray, shared_gray)
            
        # Prevent wasting CPU running detection on the exact same frame twice
        if np.array_equal(local_gray, last_processed_gray):
            delay_busywait(0.005)
            continue
            
        np.copyto(last_processed_gray, local_gray)

        # Fetch the latest attitude data
        with attitude_mutex:
            np.copyto(local_attitude, shared_attitude)

        # Extract yaw (Index 2 is Yaw in the [Roll, Pitch, Yaw] array)
        yaw_rad = local_attitude[2]

        # Run the detection
        target_pose = tracker.get_target_ned(local_gray, yaw_rad)

        # Only print if a target is actually detected
        if target_pose is not None:
            layer = target_pose["layer"].upper()
            n = target_pose["north"]
            e = target_pose["east"]
            d = target_pose["down"]
            
            print(f"TARGET LOCK [{layer}] | North: {n:+.2f}m | East: {e:+.2f}m | Down: {d:+.2f}m")
  
if __name__ == "__main__":
    import multiprocessing as mp
    from multiprocessing import shared_memory
    import time
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2 
    CALIB_BYTES = 3 * 3 * 8 
    ATTITUDE_BYTES = 3 * 8 
    POSITION_BYTES = 3 * 8 
    LOCAL_POSITION_NED_BYTES = 3 * 8
    BOOL_BYTES = 2
    TARGET_BYTES = 4 * 8

    print("VIO tester allocating shared memory...")
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
    print("VIO tester finished allocating shared memory...")

    rgb_frame_mutex = mp.Lock()
    gray_frame_mutex = mp.Lock()
    depth_frame_mutex = mp.Lock()
    attitude_mutex = mp.Lock()
    position_mutex = mp.Lock()
    local_position_ned_mutex = mp.Lock()
    slam_trigger_mutex = mp.Lock()
    slam_enabled_mutex = mp.Lock()

    from vioslam.slam import slam
    from vioslam.vio import vio
    from vioslam.broadcaster import broadcaster
    broadcaster_process = mp.Process(target=broadcaster, args=(rgb_frame_mutex, gray_frame_mutex, depth_frame_mutex, attitude_mutex, local_position_ned_mutex,))
    vio_process = mp.Process(target=vio, args=(gray_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_trigger_mutex,))
    slam_process = mp.Process(target=slam, args=(rgb_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_enabled_mutex, slam_trigger_mutex,))
    main_process = mp.Process(target=main, args=(gray_frame_mutex, depth_frame_mutex, attitude_mutex,))

    try:
        broadcaster_process.start()
        time.sleep(3)
        vio_process.start()
        time.sleep(3)
        slam_process.start()
        time.sleep(5)
        main_process.start()
        time.sleep(3)
        main_process.join()
        
    except KeyboardInterrupt:
        print("VIO tester caught keyboard interrupt. Shutting down...")
    finally:
        broadcaster_process.terminate()
        vio_process.terminate()
        slam_process.terminate()
        main_process.terminate()

        broadcaster_process.join()
        vio_process.join()
        slam_process.join()
        main_process.join()

        print("VIO tester cleaning up shared memory...")
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
        print("VIO tester processes terminated safely.")