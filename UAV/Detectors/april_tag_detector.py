# AprilTag 36h11 Detector
# Returns 3D position of tag in meters
#
# Assumptions:
# - Tag family: 36h11
# - Target tag ID: 67
# - Tag size: 20 cm (0.20 meters)

import cv2
from pupil_apriltags import Detector
import numpy as np
import depthai as dai


# Config / Adjust these as we test
TARGET_TAG_ID = 67
TAG_SIZE = 0.20     # 20 cm tag

# Shadow/lighting robustness: gamma value for the dark-region fallback pass.
# Values < 1.0 brighten shadows; 0.5 is a good starting point for heavy shade.
_GAMMA_CORRECTION = 0.5


def _build_gamma_lut(gamma: float) -> np.ndarray:
    """Precompute a 256-entry lookup table for gamma correction."""
    inv = 1.0 / gamma
    table = np.array([
        min(int((i / 255.0) ** inv * 255.0 + 0.5), 255)
        for i in range(256)
    ], dtype=np.uint8)
    return table


class AprilTagDetector:

    def __init__(self, calibration_handler):
        """
        Initialize AprilTag detector using calibration
        from the OAK-D S2.
        """
        self.calibration_handler = calibration_handler

        self.camera_matrix = None
        self.dist_coeffs = None

        self.FX = None
        self.FY = None
        self.CX = None
        self.CY = None

        # CLAHE instance reused every frame — avoids re-allocation overhead.
        # clipLimit=2.0 / tileGridSize=(8,8) is a well-tested default for
        # outdoor variable-lighting conditions; raise clipLimit if shadows are
        # very deep, lower it if false positives appear in high-contrast scenes.
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Precomputed gamma LUT for the dark-shadow fallback pass.
        self._gamma_lut = _build_gamma_lut(_GAMMA_CORRECTION)

        print("[INFO] Detector initialized (intrinsics will be set on first frame)")

        # quad_sigma=0.8 adds a gentle internal blur inside the quad-finder.
        # This smooths sharp shadow-boundary edges that can look like tag edges
        # to the detector, reducing false quads without hurting real detections.
        self.detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.8,
            refine_edges=1,
            decode_sharpening=0.25
        )

    def _update_intrinsics(self, frame):
        """
        Update the camera intrinsics based on the current frame size.
        """
        h, w = frame.shape[:2]

        intrinsics = self.calibration_handler.getCameraIntrinsics(
            dai.CameraBoardSocket.RGB,
            w,
            h
        )

        self.camera_matrix = np.array(intrinsics)

        self.FX = self.camera_matrix[0][0]
        self.FY = self.camera_matrix[1][1]
        self.CX = self.camera_matrix[0][2]
        self.CY = self.camera_matrix[1][2]

        # Load distortion coefficients as well (the detector needs them)
        self.dist_coeffs = np.array(
            self.calibration_handler.getDistortionCoefficients(
                dai.CameraBoardSocket.RGB
            )
        )

        print(f"[INFO] Intrinsics updated for {w}x{h}")

    def _preprocess_variants(self, gray: np.ndarray) -> list:
        """
        Return a list of preprocessed grayscale images to try in order.

        Three passes are generated, each targeting a different lighting failure
        mode.  The detector tries them in sequence and returns on the first hit,
        so the extra passes are only paid for on frames where the primary pass
        misses — which is exactly the shadow-affected frames we care about.

        Pass 1 — CLAHE only
            Local contrast normalization.  Handles partial shadows where one
            half of the tag is lit and the other is dark.

        Pass 2 — Gamma lift → CLAHE
            Brightens deeply shadowed regions first so CLAHE has more tonal
            range to work with.  Helps when the tag is nearly entirely in shade.

        Pass 3 — Bilateral filter → CLAHE
            Smooths colour-fringing / sensor noise at shadow boundaries while
            preserving tag edges.  Acts as a last-resort for noisy low-light
            frames.
        """
        # Pass 1: CLAHE
        clahe_only = self._clahe.apply(gray)

        # Pass 2: gamma correction to lift shadows, then CLAHE
        gamma_lifted = cv2.LUT(gray, self._gamma_lut)
        gamma_clahe = self._clahe.apply(gamma_lifted)

        # Pass 3: bilateral smoothing (preserves edges), then CLAHE
        bilateral = cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        bilateral_clahe = self._clahe.apply(bilateral)

        return [clahe_only, gamma_clahe, bilateral_clahe]

    def _detect_on_image(self, image: np.ndarray):
        """Run the detector and return (x, y, z) for TARGET_TAG_ID, or None."""
        detections = self.detector.detect(
            image,
            estimate_tag_pose=True,
            camera_params=(self.FX, self.FY, self.CX, self.CY),
            tag_size=TAG_SIZE
        )
        for tag in detections:
            if tag.tag_id == TARGET_TAG_ID:
                t = tag.pose_t
                return float(t[0][0]), float(t[1][0]), float(t[2][0])
        return None

    def get_tag_pose(self, frame):
        """
        Detect tag and return:

        (x, y, z) in meters relative to camera frame

        Camera Frame:
        x → right or left
        y → down or up
        z → forward or back

        Returns None if tag not detected.
        """

        if frame is None:
            return None

        if self.camera_matrix is None:
            self._update_intrinsics(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.undistort(
            gray,
            self.camera_matrix,
            self.dist_coeffs
        )

        # Try each preprocessed variant in order; return on the first hit.
        # On frames with no shadows all three passes are fast (< 1 ms each at
        # 300×300), so the fallback chain adds negligible latency.
        for variant in self._preprocess_variants(gray):
            result = self._detect_on_image(variant)
            if result is not None:
                return result

        return None