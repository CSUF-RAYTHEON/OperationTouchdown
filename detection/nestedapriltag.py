import cv2
import numpy as np
import cv2.aruco as aruco

class LandingTargetTracker:
    def __init__(self, camera_matrix_3x3):
        # 1. Configured Constants
        self.OUTER_ID = 77      # Match to your printed board
        self.INNER_ID = 12      # Fixed inner ID
        self.OUTER_SIZE = 0.34  # Meters
        self.INNER_SIZE = 0.09  # Meters

        # 2. Pre-build the ArUco Detector Memory
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(dictionary, params)

        # 3. Pre-build 3D Object Geometries
        def make_obj_pts(size):
            half = size / 2.0
            return np.array([
                [-half,  half, 0.0], [ half,  half, 0.0],
                [ half, -half, 0.0], [-half, -half, 0.0],
            ], dtype=np.float32)
            
        self.obj_inner = make_obj_pts(self.INNER_SIZE)
        self.obj_outer = make_obj_pts(self.OUTER_SIZE)
        
        # 4. Save Camera Intrinsics
        self.camera_matrix = camera_matrix_3x3
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _extract_pose(self, corners, obj_points, yaw_rad):
        """Internal helper to convert 2D pixels to 3D NED coordinates."""
        success, rvec, tvec = cv2.solvePnP(
            obj_points, corners.astype(np.float32),
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success: 
            return None
        
        # Camera axes to Drone axes (Forward, Right, Down)
        cam_x, cam_y, cam_z = tvec[0][0], tvec[1][0], tvec[2][0]
        forward_m, right_m, down_m = -cam_y, cam_x, cam_z
        
        # Rotate Forward/Right into true North/East using Drone's Yaw
        north_m = forward_m * np.cos(yaw_rad) - right_m * np.sin(yaw_rad)
        east_m = forward_m * np.sin(yaw_rad) + right_m * np.cos(yaw_rad)
        
        return north_m, east_m, down_m

    def get_ned(self, frame, yaw_rad=0.0):
        """Main public method to be called in your mission control loop."""
        
        # Safely ensure the frame is 2D grayscale
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ==========================================
        # PASS 1: Standard Detection
        # ==========================================
        corners1, ids1, _ = self.detector.detectMarkers(gray)
        found = {}
        if ids1 is not None:
            for i, tid in enumerate(ids1.flatten()):
                found[tid] = corners1[i].reshape(4, 2)

        # Priority 1: Inner Tag
        if self.INNER_ID in found:
            pose = self._extract_pose(found[self.INNER_ID], self.obj_inner, yaw_rad)
            if pose: return {"layer": "inner", "north": pose[0], "east": pose[1], "down": pose[2]}

        # Priority 2: Outer Tag (if it happens to be uncorrupted)
        if self.OUTER_ID in found:
            pose = self._extract_pose(found[self.OUTER_ID], self.obj_outer, yaw_rad)
            if pose: return {"layer": "outer", "north": pose[0], "east": pose[1], "down": pose[2]}

        # ==========================================
        # PASS 2: Masking Recovery for Outer Tag
        # ==========================================
        if ids1 is not None:
            masked = gray.copy()
            for c in corners1:
                pts = c.reshape((-1, 2)).astype(np.float32)
                center = pts.mean(axis=0)

                ring_outer = (center + 1.35 * (pts - center)).astype(np.int32)
                ring_inner = (center + 1.10 * (pts - center)).astype(np.int32)
                ring_mask = np.zeros_like(gray)
                cv2.fillPoly(ring_mask, [ring_outer], 255)
                cv2.fillPoly(ring_mask, [ring_inner], 0)
                
                fill_val = int(np.median(gray[ring_mask == 255])) if np.any(ring_mask) else 255
                fill_poly = (center + 1.25 * (pts - center)).astype(np.int32)
                cv2.fillPoly(masked, [fill_poly], color=fill_val)

            corners2, ids2, _ = self.detector.detectMarkers(masked)
            if ids2 is not None:
                for i, tid in enumerate(ids2.flatten()):
                    if tid == self.OUTER_ID:
                        pose = self._extract_pose(corners2[i].reshape(4, 2), self.obj_outer, yaw_rad)
                        if pose: return {"layer": "outer", "north": pose[0], "east": pose[1], "down": pose[2]}

        return None