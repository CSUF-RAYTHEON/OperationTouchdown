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


def _division_normalize(img: np.ndarray, blur_ksize: int = 71) -> np.ndarray:
    """
    Remove large-scale illumination gradients by dividing each pixel by a
    heavily blurred (local mean) version of the image, then re-centering the
    result around 128.

    This is the primary fix for backlight: a bright background is a large-scale
    gradient, and dividing it out reveals the tag's local contrast regardless of
    whether the background is much brighter than the tag.  It also handles
    uneven shadows that fall across only part of the frame.

    blur_ksize controls how large a gradient is removed; 71 pixels at 300×300
    captures most real-world lighting gradients without erasing the tag itself
    (which is ~50–100 pixels wide).
    """
    if blur_ksize % 2 == 0:
        blur_ksize += 1
    local_mean = cv2.GaussianBlur(
        img, (blur_ksize, blur_ksize), 0
    ).astype(np.float32) + 1.0
    normalized = np.clip(
        img.astype(np.float32) / local_mean * 128.0, 0, 255
    ).astype(np.uint8)
    return normalized


def _adaptive_thresh(img: np.ndarray, block_size: int, c: int = 5) -> np.ndarray:
    """
    Convert the image to a near-binary representation using Gaussian adaptive
    thresholding.  Each pixel is compared only to its local neighbourhood of
    size block_size × block_size, so the output is immune to absolute
    brightness: a tag sitting in pitch-black shadow looks identical to one in
    full sunlight as long as the white/black squares differ even slightly from
    their immediate surroundings.

    block_size must be odd and > 1.
    c is subtracted from the local mean before thresholding; positive values
    make the threshold stricter (fewer pixels become 'white').

    Two block sizes are used in the pipeline:
      - Small  (31)  — recovers fine tag detail in localised shadows.
      - Large  (71)  — handles gradual, wide shadow gradients and backlight
                       halos where the brightness ramp is spread over many pixels.
    """
    if block_size % 2 == 0:
        block_size += 1
    return cv2.adaptiveThreshold(
        img, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size,
        c,
    )


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

        ── Shadow / under-exposure tier ───────────────────────────────────────

        Pass 1   CLAHE (standard)
                 Local contrast normalisation.  Handles partial shadows where
                 one side of the tag is lit and the other is dark.

        Pass 2   Gamma 2.2× → standard CLAHE
                 Mild brightness lift for lightly shaded regions.

        Pass 3   Bilateral → standard CLAHE
                 Smooths sensor noise at shadow boundaries while preserving
                 tag edges.

        Pass 4   Division normalise → aggressive CLAHE
                 Removes large-scale illumination gradients by dividing out the
                 local mean.  Works for both gradual shadows and backlighting
                 and is intentionally placed early because it is effective
                 across many conditions with minimal distortion.

        Pass 5   Gamma 4.0× → aggressive CLAHE
                 Deep-shadow primary: pixel at 20 → ~120.

        Pass 6   Gamma 4.0× → bilateral → aggressive CLAHE
                 Same deep lift but de-noised first for noisy dark frames.

        Pass 7   Gamma 6.0× → aggressive CLAHE   [mid-dark fill]
                 Covers the range between deep shadow and near-black.

        Pass 8   Gamma 6.0× → division normalise → aggressive CLAHE
                 Lifts the dark region then removes any remaining gradient.
                 Targets the case where a shadow gradient still exists after
                 brightening.

        Pass 9   Gamma 8.0× → aggressive CLAHE
                 Near-black primary: pixel at 5 → ~152.

        Pass 10  Percentile stretch → aggressive CLAHE
                 Maps the actual tonal range (even 2–18) to full 0–255 before
                 CLAHE.  Most powerful single near-black pass.

        Pass 11  Gaussian denoise → percentile stretch → aggressive CLAHE
                 Same stretch but de-noised first.  Best for very dark, noisy
                 frames.

        Pass 12  Adaptive threshold, blockSize=31
                 Completely ignores absolute brightness; each pixel is only
                 compared to its local 31×31 neighbourhood.  Recovers the tag
                 pattern from localised deep shadows as long as any relative
                 contrast remains between the white and black squares.

        Pass 13  Adaptive threshold, blockSize=71
                 Larger neighbourhood handles wide, gradual shadow gradients
                 and broad backlight halos that span much of the frame.

        ── Glare / backlight / over-exposure tier ─────────────────────────────
        Gamma < 1 compresses bright pixels back into a workable range.
        Division normalise removes the large bright-background gradient that is
        the defining feature of backlight.

        Pass 14  Division normalise → standard CLAHE   [backlight primary]
                 Same division normalise used in Pass 4 but listed again here
                 so it is also attempted from the bright side when the shadow
                 passes have all failed.  This is the most effective single
                 pass for backlit tags.

        Pass 15  Gamma 0.5 → standard CLAHE   [mild over-exposure]
                 Gentle global darkening for slight glare or direct sunlight
                 reflecting off the landing pad.  Pixel at 230 → ~207.

        Pass 16  Gamma 0.5 → division normalise → aggressive CLAHE
                 Combines global highlight compression with local gradient
                 removal.  Handles backlit scenes where the tag is somewhat
                 visible but washed into the background.

        Pass 17  Gamma 0.3 → aggressive CLAHE   [moderate over-exposure]
                 Moderate global darkening; pixel at 230 → ~176.

        Pass 18  Gamma 0.15 → aggressive CLAHE   [strong / near-white]
                 Heavy darkening for near-white or heavily washed-out frames.
                 Pixel at 230 → ~131.

        Pass 19  Gaussian denoise → gamma 0.3 → aggressive CLAHE
                 De-noised moderate darkening for bright, noisy frames.

        Pass 20  Adaptive threshold on gamma 0.3 darkened image
                 Runs adaptive thresholding after global highlight compression.
                 Recovers near-white frames where the tag structure is locally
                 distinguishable but globally washed out.
        """
        # ── Shadow tier ────────────────────────────────────────────────────

        # Pass 1
        p1 = self._clahe.apply(gray)

        # Pass 2
        p2 = self._clahe.apply(cv2.LUT(gray, self._gamma_lut))

        # Pass 3
        p3 = self._clahe.apply(
            cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        )

        # Pass 4 — division normalise (general gradient removal)
        div_norm = _division_normalize(gray)
        p4 = self._clahe_deep.apply(div_norm)

        # Pass 5
        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        p5 = self._clahe_deep.apply(gamma_strong)

        # Pass 6
        p6 = self._clahe_deep.apply(
            cv2.bilateralFilter(gamma_strong, d=9, sigmaColor=75, sigmaSpace=75)
        )

        # Pass 7 — mid-dark fill
        gamma_mid_dark = cv2.LUT(gray, self._gamma_lut_mid_dark)
        p7 = self._clahe_deep.apply(gamma_mid_dark)

        # Pass 8 — mid-dark lift then gradient removal
        p8 = self._clahe_deep.apply(_division_normalize(gamma_mid_dark))

        # Pass 9
        gamma_extreme = cv2.LUT(gray, self._gamma_lut_extreme)
        p9 = self._clahe_deep.apply(gamma_extreme)

        # Pass 10
        p10 = self._clahe_deep.apply(_percentile_stretch(gray))

        # Pass 11
        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p11 = self._clahe_deep.apply(_percentile_stretch(denoised))

        # Pass 12 — adaptive threshold, small neighbourhood
        p12 = _adaptive_thresh(gray, block_size=31)

        # Pass 13 — adaptive threshold, large neighbourhood (wide gradients)
        p13 = _adaptive_thresh(gray, block_size=71)

        # ── Glare / backlight / over-exposure tier ─────────────────────────

        # Pass 14 — division normalise as backlight primary (tried again here
        #           so the bright-side pipeline also benefits from it)
        p14 = self._clahe.apply(div_norm)

        # Pass 15
        white_mild = cv2.LUT(gray, self._gamma_lut_white_mild)
        p15 = self._clahe.apply(white_mild)

        # Pass 16 — mild global compression + local gradient removal
        p16 = self._clahe_deep.apply(_division_normalize(white_mild))

        # Pass 17
        white_moderate = cv2.LUT(gray, self._gamma_lut_white_moderate)
        p17 = self._clahe_deep.apply(white_moderate)

        # Pass 18
        p18 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_white_strong))

        # Pass 19
        bright_denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p19 = self._clahe_deep.apply(
            cv2.LUT(bright_denoised, self._gamma_lut_white_moderate)
        )

        # Pass 20 — adaptive threshold on moderately darkened image
        p20 = _adaptive_thresh(white_moderate, block_size=31)

        return [
            p1,  p2,  p3,  p4,  p5,
            p6,  p7,  p8,  p9,  p10,
            p11, p12, p13,
            p14, p15, p16, p17, p18, p19, p20,
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