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


def _unsharp_mask(img: np.ndarray, sigma: float = 2.0, strength: float = 1.5) -> np.ndarray:
    """
    Sharpen the image using unsharp masking:
        output = img + strength × (img − blur(img))

    This is applied AFTER normalisation passes to give the AprilTag corner
    finder crisper, better-defined tag edges.  Spotty / intermittent detection
    is often caused by soft edges that the quad-finder finds on some frames but
    not others; sharpening makes the response consistent across frames.

    sigma=2.0 sharpens at the scale of a tag edge (~2 px) without creating
    large ringing halos.  strength=1.5 adds the high-frequency detail at
    150% amplitude — aggressive enough to recover soft edges but below the
    point where noise starts to dominate.
    """
    blurred = cv2.GaussianBlur(img, (0, 0), sigmaX=sigma)
    return cv2.addWeighted(img, 1.0 + strength, blurred, -strength, 0)


def _local_std_normalize(
    img: np.ndarray, ksize: int = 31, scale: float = 48.0
) -> np.ndarray:
    """
    Normalize each pixel against the local standard deviation of its
    neighbourhood.

    Unlike division normalization (which only removes the local mean / DC
    component), this also compresses bright patches and lifts dark ones by
    dividing out the local contrast magnitude.  The result is that dappled
    light — alternating bright and dark patches caused by foliage, reflective
    surfaces, or uneven ground cover — is equalized so every part of the frame
    has similar contrast range regardless of local brightness.

    Formula:
        local_mean = GaussianBlur(img, ksize)
        local_std  = sqrt(GaussianBlur(img², ksize) − local_mean²) + 1
        output     = (img − local_mean) / local_std × scale + 128

    ksize=31 captures lighting patches wider than ~30 px without erasing the
    tag pattern (~50–100 px wide).  scale=48 centers the output at 128 with a
    comfortable ±48 range for typical outdoor contrast levels.
    """
    img_f = img.astype(np.float32)
    k = (ksize, ksize)
    local_mean   = cv2.GaussianBlur(img_f, k, 0)
    local_sq_mean = cv2.GaussianBlur(img_f * img_f, k, 0)
    variance     = local_sq_mean - local_mean * local_mean
    local_std    = np.sqrt(np.maximum(variance, 0.0)) + 1.0
    normalized   = (img_f - local_mean) / local_std * scale + 128.0
    return np.clip(normalized, 0, 255).astype(np.uint8)


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
            dai.CameraBoardSocket.CAM_A,
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
                dai.CameraBoardSocket.CAM_A
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

        Pass 2   CLAHE → unsharp mask
                 Same CLAHE output but edge-sharpened.  Directly targets
                 "spotty" detection: if CLAHE sees the tag but the corner
                 finder misses it on some frames, sharpening makes the edges
                 consistent enough to always lock on.

        Pass 3   Gamma 2.2× → standard CLAHE
                 Mild brightness lift for lightly shaded regions.

        Pass 4   Bilateral → standard CLAHE
                 Smooths sensor noise at shadow boundaries while preserving
                 tag edges.

        Pass 5   Division normalise → aggressive CLAHE
                 Removes large-scale illumination gradients.  Works for both
                 gradual shadows and backlighting.

        Pass 6   Division normalise → unsharp mask → aggressive CLAHE
                 Same gradient removal but with edge sharpening applied after.
                 This is the primary fix for lighting conditions where the tag
                 is visible but detection is inconsistent: the div-norm makes
                 the image even, unsharp mask makes the edges crisp.

        Pass 7   Local std normalise → aggressive CLAHE   [dappled-light primary]
                 Normalises by local standard deviation, not just local mean.
                 Equalizes both bright patches and dark patches simultaneously,
                 which is the defining feature of dappled or mixed light.

        Pass 8   Local std normalise → unsharp mask → aggressive CLAHE
                 Same dappled-light normalization with edge sharpening on top.
                 Most consistent pass for any mixed-illumination condition.

        Pass 9   Gamma 4.0× → aggressive CLAHE
                 Deep-shadow primary: pixel at 20 → ~120.

        Pass 10  Gamma 4.0× → bilateral → aggressive CLAHE
                 Same deep lift but de-noised first for noisy dark frames.

        Pass 11  Gamma 6.0× → aggressive CLAHE   [mid-dark fill]
                 Covers the range between deep shadow and near-black.

        Pass 12  Gamma 6.0× → division normalise → aggressive CLAHE
                 Lifts the dark region then removes any remaining gradient.

        Pass 13  Gamma 8.0× → aggressive CLAHE
                 Near-black primary: pixel at 5 → ~152.

        Pass 14  Percentile stretch → aggressive CLAHE
                 Maps the actual tonal range (even 2–18) to full 0–255.

        Pass 15  Gaussian denoise → percentile stretch → aggressive CLAHE
                 Same stretch but de-noised first for very dark noisy frames.

        Pass 16  Adaptive threshold, blockSize=31
                 Completely ignores absolute brightness; detects local contrast
                 only.  Works even in near-total shadow.

        Pass 17  Adaptive threshold, blockSize=71
                 Larger neighbourhood for wide, gradual gradients and backlight.

        ── Mixed / dappled light tier ─────────────────────────────────────────

        Pass 18  Denoised → local std normalise → aggressive CLAHE
                 Same as Pass 7 but de-noised first.  Best for outdoor frames
                 with both sensor noise and dappled lighting (common under
                 partial cloud cover or through foliage).

        ── Glare / backlight / over-exposure tier ─────────────────────────────

        Pass 19  Division normalise → standard CLAHE   [backlight primary]

        Pass 20  Gamma 0.5 → standard CLAHE   [mild over-exposure]

        Pass 21  Gamma 0.5 → unsharp mask → standard CLAHE
                 Mild darkening with edge sharpening.  Targets bright scenes
                 where detection is inconsistent because tag edges are soft
                 after highlight compression.

        Pass 22  Gamma 0.5 → division normalise → aggressive CLAHE
                 Combines global highlight compression with local gradient
                 removal.

        Pass 23  Local std normalise on gamma 0.5 image → aggressive CLAHE
                 Dappled-light normalization from the bright side.  Handles
                 frames with mixed glare and shadow simultaneously.

        Pass 24  Gamma 0.3 → aggressive CLAHE   [moderate over-exposure]

        Pass 25  Gamma 0.15 → aggressive CLAHE   [strong / near-white]

        Pass 26  Gaussian denoise → gamma 0.3 → aggressive CLAHE

        Pass 27  Adaptive threshold on gamma 0.3 darkened image
        """
        # ── Shadow tier ────────────────────────────────────────────────────

        # Pass 1
        p1 = self._clahe.apply(gray)

        # Pass 2 — CLAHE + sharpening (spotty-detection fix)
        p2 = _unsharp_mask(p1)

        # Pass 3
        p3 = self._clahe.apply(cv2.LUT(gray, self._gamma_lut))

        # Pass 4
        p4 = self._clahe.apply(
            cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        )

        # Pass 5 — division normalise
        div_norm = _division_normalize(gray)
        p5 = self._clahe_deep.apply(div_norm)

        # Pass 6 — division normalise + sharpening
        p6 = self._clahe_deep.apply(_unsharp_mask(div_norm))

        # Pass 7 — local std normalise (dappled-light primary)
        lsdn = _local_std_normalize(gray)
        p7 = self._clahe_deep.apply(lsdn)

        # Pass 8 — local std normalise + sharpening
        p8 = self._clahe_deep.apply(_unsharp_mask(lsdn))

        # Pass 9
        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        p9 = self._clahe_deep.apply(gamma_strong)

        # Pass 10
        p10 = self._clahe_deep.apply(
            cv2.bilateralFilter(gamma_strong, d=9, sigmaColor=75, sigmaSpace=75)
        )

        # Pass 11 — mid-dark fill
        gamma_mid_dark = cv2.LUT(gray, self._gamma_lut_mid_dark)
        p11 = self._clahe_deep.apply(gamma_mid_dark)

        # Pass 12 — mid-dark lift + gradient removal
        p12 = self._clahe_deep.apply(_division_normalize(gamma_mid_dark))

        # Pass 13
        p13 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_extreme))

        # Pass 14
        p14 = self._clahe_deep.apply(_percentile_stretch(gray))

        # Pass 15
        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p15 = self._clahe_deep.apply(_percentile_stretch(denoised))

        # Pass 16 — adaptive threshold, small neighbourhood
        p16 = _adaptive_thresh(gray, block_size=31)

        # Pass 17 — adaptive threshold, large neighbourhood
        p17 = _adaptive_thresh(gray, block_size=71)

        # ── Mixed / dappled light tier ──────────────────────────────────────

        # Pass 18 — denoised local std normalise
        p18 = self._clahe_deep.apply(_local_std_normalize(denoised))

        # ── Glare / backlight / over-exposure tier ──────────────────────────

        # Pass 19 — div norm as backlight primary
        p19 = self._clahe.apply(div_norm)

        # Pass 20
        white_mild = cv2.LUT(gray, self._gamma_lut_white_mild)
        p20 = self._clahe.apply(white_mild)

        # Pass 21 — mild darkening + sharpening
        p21 = self._clahe.apply(_unsharp_mask(white_mild))

        # Pass 22 — mild darkening + gradient removal
        p22 = self._clahe_deep.apply(_division_normalize(white_mild))

        # Pass 23 — local std normalise on mildly darkened image
        p23 = self._clahe_deep.apply(_local_std_normalize(white_mild))

        # Pass 24
        white_moderate = cv2.LUT(gray, self._gamma_lut_white_moderate)
        p24 = self._clahe_deep.apply(white_moderate)

        # Pass 25
        p25 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_white_strong))

        # Pass 26
        bright_denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p26 = self._clahe_deep.apply(
            cv2.LUT(bright_denoised, self._gamma_lut_white_moderate)
        )

        # Pass 27 — adaptive threshold on moderately darkened image
        p27 = _adaptive_thresh(white_moderate, block_size=31)

        return [
            p1,  p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9,
            p10, p11, p12, p13, p14, p15, p16, p17,
            p18,
            p19, p20, p21, p22, p23, p24, p25, p26, p27,
        ]

    def _detect_raw(self, image: np.ndarray):
        """
        Run the detector on a preprocessed image and return the raw Detection
        object for TARGET_TAG_ID, or None.  Callers that only need pose should
        use the returned object's pose_t; callers that need corners/center for
        visualization use corners and center directly.
        """
        detections = self.detector.detect(
            image,
            estimate_tag_pose=True,
            camera_params=(self.FX, self.FY, self.CX, self.CY),
            tag_size=TAG_SIZE
        )
        for tag in detections:
            if tag.tag_id == TARGET_TAG_ID:
                return tag
        return None

    def _preprocess_and_find(self, gray: np.ndarray):
        """
        Iterate preprocessing variants in order and return the first raw
        Detection object that succeeds, or None if all variants fail.
        """
        for variant in self._preprocess_variants(gray):
            tag = self._detect_raw(variant)
            if tag is not None:
                return tag
        return None

    def _prepare_gray(self, frame):
        """Convert frame to undistorted grayscale, updating intrinsics if needed."""
        if self.camera_matrix is None:
            self._update_intrinsics(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.undistort(gray, self.camera_matrix, self.dist_coeffs)

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
        tag = self._preprocess_and_find(self._prepare_gray(frame))
        if tag is None:
            return None
        t = tag.pose_t
        return float(t[0][0]), float(t[1][0]), float(t[2][0])

    def get_tag_detection(self, frame):
        """
        Detect tag and return the raw Detection object for TARGET_TAG_ID,
        or None if the tag is not found.

        The Detection object exposes:
          .tag_id   — integer tag ID
          .corners  — (4, 2) float32 array of corner pixel coordinates
          .center   — (2,) float32 center pixel coordinate
          .pose_t   — (3, 1) translation vector in meters (camera frame)
          .pose_R   — (3, 3) rotation matrix

        Use this method when you need corners/center for visualization in
        addition to the pose.  get_tag_pose() is sufficient when only the
        3-D position is needed.
        """
        if frame is None:
            return None
        return self._preprocess_and_find(self._prepare_gray(frame))