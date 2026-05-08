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

# Gamma uses the photographic convention:
#   output = (pixel / 255) ^ (1 / gamma) × 255
#
#  gamma > 1.0  →  BRIGHTENS dark pixels  (shadow / under-exposure tier)
#  gamma < 1.0  →  DARKENS  bright pixels (glare / over-exposure tier)
#
# ── Shadow / under-exposure tier (gamma > 1) ────────────────────────────────
# _GAMMA_MODERATE   partial shadow, one side of the tag in shade
# _GAMMA_DEEP       tag mostly in deep shadow
# _GAMMA_MID_DARK   fills the gap between deep shadow and near-black
# _GAMMA_EXTREME    near-black; pixel at value 5 lifts to ~152
_GAMMA_MODERATE = 2.2
_GAMMA_DEEP     = 4.0
_GAMMA_MID_DARK = 6.0
_GAMMA_EXTREME  = 8.0

# ── Glare / over-exposure tier (gamma < 1) ──────────────────────────────────
# _GAMMA_WHITE_MILD     mild glare / slight over-exposure
# _GAMMA_WHITE_MODERATE moderate over-exposure; blacks appear mid-gray
# _GAMMA_WHITE_STRONG   heavy over-exposure / near-white frames
_GAMMA_WHITE_MILD     = 0.5
_GAMMA_WHITE_MODERATE = 0.3
_GAMMA_WHITE_STRONG   = 0.15

# Percentile clipping for the histogram-stretch passes.
# Clips the bottom/top N% of pixels before stretching to avoid letting
# a handful of noise spikes collapse the entire tonal range.
_STRETCH_LOW_PCT  = 1.0
_STRETCH_HIGH_PCT = 99.0


def _build_gamma_lut(gamma: float) -> np.ndarray:
    """
    Precompute a 256-entry brightening lookup table.
    Photographic convention: gamma > 1 brightens dark pixels.
      output = (i / 255) ^ (1 / gamma) × 255
    """
    inv = 1.0 / gamma
    table = np.array(
        [min(int((i / 255.0) ** inv * 255.0 + 0.5), 255) for i in range(256)],
        dtype=np.uint8,
    )
    return table


def _percentile_stretch(img: np.ndarray) -> np.ndarray:
    """
    Stretch the pixel value range so that the _STRETCH_LOW_PCT percentile
    maps to 0 and the _STRETCH_HIGH_PCT percentile maps to 255.

    This is the most powerful tool for near-black frames: even if all pixels
    are in the range [2, 18], whatever relative contrast exists between the
    tag's white and black squares gets expanded to fill the full 0–255 range
    before CLAHE runs on top of it.

    Percentile clipping prevents a single bright noise spike from collapsing
    the stretch and making the rest of the image look uniformly dark.
    """
    lo = np.percentile(img, _STRETCH_LOW_PCT)
    hi = np.percentile(img, _STRETCH_HIGH_PCT)
    if hi <= lo:
        return img
    stretched = np.clip(
        (img.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255
    ).astype(np.uint8)
    return stretched


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

        # Standard CLAHE — handles partial / mild shadows.
        # clipLimit=2.0, tileGridSize=(8,8) is the well-tested outdoor default.
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Aggressive CLAHE — used for deep-shadow passes.
        # Higher clipLimit (5.0) pushes more contrast amplification.
        # Smaller tileGridSize (4,4) normalizes at a finer scale, which
        # recovers tag structure when the shadow covers most of the tag.
        self._clahe_deep = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))

        # Shadow tier LUTs (gamma > 1, brighten)
        self._gamma_lut          = _build_gamma_lut(_GAMMA_MODERATE)
        self._gamma_lut_strong   = _build_gamma_lut(_GAMMA_DEEP)
        self._gamma_lut_mid_dark = _build_gamma_lut(_GAMMA_MID_DARK)
        self._gamma_lut_extreme  = _build_gamma_lut(_GAMMA_EXTREME)

        # Glare / over-exposure tier LUTs (gamma < 1, darken)
        self._gamma_lut_white_mild     = _build_gamma_lut(_GAMMA_WHITE_MILD)
        self._gamma_lut_white_moderate = _build_gamma_lut(_GAMMA_WHITE_MODERATE)
        self._gamma_lut_white_strong   = _build_gamma_lut(_GAMMA_WHITE_STRONG)

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
        Return preprocessed grayscale images to try in order, from least to
        most aggressive.  Early passes exit quickly on normal frames; the
        expensive passes only execute when all lighter passes have failed.

        ── Shadow / under-exposure tier (dark frames) ─────────────────────────

        Pass 1  CLAHE (standard)
                Local contrast normalisation.  Handles partial shadows where
                one side of the tag is lit and the other is dark.

        Pass 2  Gamma 2.2× → standard CLAHE
                Mild brightness lift before CLAHE for lightly shaded regions.

        Pass 3  Bilateral → standard CLAHE
                Smooths sensor noise at shadow boundaries while preserving
                tag edges.  Useful for low-light / high-gain frames.

        Pass 4  Gamma 4.0× → aggressive CLAHE
                Deep-shadow primary: pixel at 20 → ~120.

        Pass 5  Gamma 4.0× → bilateral → aggressive CLAHE
                Same deep lift but de-noised first for noisy dark frames.

        Pass 6  Gamma 6.0× → aggressive CLAHE   [mid-dark fill]
                Covers the range between deep shadow and near-black where
                gamma 4.0 undershoots and gamma 8.0 can over-smooth.

        Pass 7  Gamma 8.0× → aggressive CLAHE
                Near-black primary: pixel at 5 → ~152.

        Pass 8  Percentile stretch → aggressive CLAHE
                Maps the actual tonal range of the frame (even 2–18) to
                full 0–255 before CLAHE.  Most powerful near-black pass.

        Pass 9  Gaussian denoise → percentile stretch → aggressive CLAHE
                Same stretch but de-noised first so noise spikes don't
                dominate the stretched range.  Best for very dark noisy frames.

        ── Glare / over-exposure tier (bright frames) ─────────────────────────
        When a frame is over-exposed the tag's black squares lift to mid-gray,
        washing out the contrast.  Gamma < 1 compresses bright pixels back into
        a workable range before CLAHE restores local contrast.

        Pass 10  Gamma 0.5 → standard CLAHE   [mild over-exposure]
                 Gentle darkening; handles slight glare or direct sunlight
                 reflecting off the landing pad.  Pixel at 230 → ~207.

        Pass 11  Gamma 0.3 → aggressive CLAHE  [moderate over-exposure]
                 Moderate darkening; handles scenes where blacks appear gray.
                 Pixel at 230 → ~176.

        Pass 12  Gamma 0.15 → aggressive CLAHE  [strong / near-white]
                 Heavy darkening for near-white or heavily washed-out frames.
                 Pixel at 230 → ~131.

        Pass 13  Gaussian denoise → gamma 0.3 → aggressive CLAHE
                 Same moderate darkening but de-noised first.  Handles bright,
                 high-sensor-gain frames where noise was amplified by exposure.
        """
        # ── Shadow tier ────────────────────────────────────────────────────
        # Pass 1
        clahe_only = self._clahe.apply(gray)

        # Pass 2
        gamma_moderate = cv2.LUT(gray, self._gamma_lut)
        gamma_moderate_clahe = self._clahe.apply(gamma_moderate)

        # Pass 3
        bilateral = cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        bilateral_clahe = self._clahe.apply(bilateral)

        # Pass 4
        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        gamma_strong_clahe_deep = self._clahe_deep.apply(gamma_strong)

        # Pass 5
        gamma_strong_bilateral = cv2.bilateralFilter(
            gamma_strong, d=9, sigmaColor=75, sigmaSpace=75
        )
        gamma_strong_bilateral_clahe_deep = self._clahe_deep.apply(gamma_strong_bilateral)

        # Pass 6 — mid-dark fill (gamma 6.0)
        gamma_mid_dark = cv2.LUT(gray, self._gamma_lut_mid_dark)
        gamma_mid_dark_clahe_deep = self._clahe_deep.apply(gamma_mid_dark)

        # Pass 7
        gamma_extreme = cv2.LUT(gray, self._gamma_lut_extreme)
        gamma_extreme_clahe_deep = self._clahe_deep.apply(gamma_extreme)

        # Pass 8
        stretched = _percentile_stretch(gray)
        stretched_clahe_deep = self._clahe_deep.apply(stretched)

        # Pass 9
        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        denoised_stretched_clahe_deep = self._clahe_deep.apply(
            _percentile_stretch(denoised)
        )

        # ── Glare / over-exposure tier ─────────────────────────────────────
        # Pass 10 — mild over-exposure
        white_mild = cv2.LUT(gray, self._gamma_lut_white_mild)
        white_mild_clahe = self._clahe.apply(white_mild)

        # Pass 11 — moderate over-exposure
        white_moderate = cv2.LUT(gray, self._gamma_lut_white_moderate)
        white_moderate_clahe_deep = self._clahe_deep.apply(white_moderate)

        # Pass 12 — strong / near-white
        white_strong = cv2.LUT(gray, self._gamma_lut_white_strong)
        white_strong_clahe_deep = self._clahe_deep.apply(white_strong)

        # Pass 13 — bright + noisy: denoise then moderate darkening
        bright_denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        bright_denoised_white_moderate = cv2.LUT(
            bright_denoised, self._gamma_lut_white_moderate
        )
        bright_denoised_clahe_deep = self._clahe_deep.apply(
            bright_denoised_white_moderate
        )

        return [
            # shadow tier
            clahe_only,
            gamma_moderate_clahe,
            bilateral_clahe,
            gamma_strong_clahe_deep,
            gamma_strong_bilateral_clahe_deep,
            gamma_mid_dark_clahe_deep,
            gamma_extreme_clahe_deep,
            stretched_clahe_deep,
            denoised_stretched_clahe_deep,
            # glare / over-exposure tier
            white_mild_clahe,
            white_moderate_clahe_deep,
            white_strong_clahe_deep,
            bright_denoised_clahe_deep,
        ]

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