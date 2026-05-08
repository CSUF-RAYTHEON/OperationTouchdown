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

# Gamma brightening uses the photographic convention:
#   output = (pixel / 255) ^ (1 / gamma) × 255
# gamma > 1.0 BRIGHTENS (lifts shadows); gamma < 1.0 darkens.
#
# _GAMMA_MODERATE — partial shadows, one side of the tag in shade
# _GAMMA_DEEP     — tag nearly or fully inside a deep shadow
# _GAMMA_EXTREME  — near-black frames; pixel at value 5 lifts to ~152
_GAMMA_MODERATE = 2.2
_GAMMA_DEEP = 4.0
_GAMMA_EXTREME = 8.0

# Percentile clipping for the histogram-stretch passes.
# Clips the bottom/top N% of pixels before stretching to avoid letting
# a handful of noise spikes collapse the entire tonal range.
_STRETCH_LOW_PCT = 1.0
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

        # Gamma LUTs — each tier lifts dark pixels progressively harder.
        self._gamma_lut = _build_gamma_lut(_GAMMA_MODERATE)
        self._gamma_lut_strong = _build_gamma_lut(_GAMMA_DEEP)
        self._gamma_lut_extreme = _build_gamma_lut(_GAMMA_EXTREME)

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

        Passes are ordered from least to most aggressive so that the common
        case (mild or no shadow) returns quickly, and the expensive deep-shadow
        passes only run when everything else has already failed.

        Pass 1 — CLAHE (standard)
            Local contrast normalization.  Handles partial shadows where one
            side of the tag is lit and the other is dark.

        Pass 2 — Moderate gamma lift (2.2×) → standard CLAHE
            Brightens pixel values before CLAHE so that the equalizer has more
            tonal range to work with in lightly-shaded regions.

        Pass 3 — Bilateral filter → standard CLAHE
            Smooths sensor noise at shadow boundaries while preserving edges.
            Useful for low-light / high-ISO frames.

        Pass 4 — Strong gamma lift (4.0×) → aggressive CLAHE
            Aggressively raises very dark pixels (a pixel at value 20 becomes
            ~120) before applying high-clip-limit CLAHE with fine tiles.
            Primary recovery pass for tags almost entirely in deep shadow.

        Pass 5 — Strong gamma lift (4.0×) → bilateral smooth → aggressive CLAHE
            Same strong lift but de-noised first.  Last resort for dark,
            noisy frames where shadow boundaries introduce false edges.

        ── Near-black / near-zero passes ──────────────────────────────────────
        The passes below are for frames where most pixel values are in roughly
        the 0–20 range.  The tag's white squares are only marginally brighter
        than its black squares, so the goal is to amplify that tiny relative
        contrast before handing off to the detector.

        Pass 6 — Extreme gamma lift (8.0×) → aggressive CLAHE
            Pushes a pixel at value 5 up to ~152.  Fastest near-black pass.

        Pass 7 — Percentile stretch → aggressive CLAHE
            Maps whatever tonal range actually exists in the frame (even if it
            is just 2–18) to the full 0–255 scale, then CLAHE enhances local
            contrast on top.  Most powerful single pass for near-black frames.

        Pass 8 — Gaussian denoise → percentile stretch → aggressive CLAHE
            De-noises before stretching so sensor noise spikes don't dominate
            the stretched range.  Best for very dark, high-gain (noisy) frames.
        """
        # Pass 1: standard CLAHE only
        clahe_only = self._clahe.apply(gray)

        # Pass 2: moderate gamma lift then standard CLAHE
        gamma_moderate = cv2.LUT(gray, self._gamma_lut)
        gamma_moderate_clahe = self._clahe.apply(gamma_moderate)

        # Pass 3: bilateral de-noise then standard CLAHE
        bilateral = cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        bilateral_clahe = self._clahe.apply(bilateral)

        # Pass 4: strong gamma lift then aggressive CLAHE (deep-shadow primary)
        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        gamma_strong_clahe_deep = self._clahe_deep.apply(gamma_strong)

        # Pass 5: strong gamma lift, de-noised, then aggressive CLAHE
        gamma_strong_bilateral = cv2.bilateralFilter(
            gamma_strong, d=9, sigmaColor=75, sigmaSpace=75
        )
        gamma_strong_bilateral_clahe_deep = self._clahe_deep.apply(gamma_strong_bilateral)

        # Pass 6: extreme gamma lift then aggressive CLAHE (near-black primary)
        gamma_extreme = cv2.LUT(gray, self._gamma_lut_extreme)
        gamma_extreme_clahe_deep = self._clahe_deep.apply(gamma_extreme)

        # Pass 7: percentile stretch then aggressive CLAHE (near-black strongest)
        stretched = _percentile_stretch(gray)
        stretched_clahe_deep = self._clahe_deep.apply(stretched)

        # Pass 8: Gaussian denoise → percentile stretch → aggressive CLAHE
        # Gaussian is cheaper than bilateral and appropriate here since we
        # want maximum noise floor reduction before the stretch amplifies it.
        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        denoised_stretched = _percentile_stretch(denoised)
        denoised_stretched_clahe_deep = self._clahe_deep.apply(denoised_stretched)

        return [
            clahe_only,
            gamma_moderate_clahe,
            bilateral_clahe,
            gamma_strong_clahe_deep,
            gamma_strong_bilateral_clahe_deep,
            gamma_extreme_clahe_deep,
            stretched_clahe_deep,
            denoised_stretched_clahe_deep,
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