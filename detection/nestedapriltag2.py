import cv2
import numpy as np
import cv2.aruco as aruco

class FastLandingTargetTracker:
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
    
