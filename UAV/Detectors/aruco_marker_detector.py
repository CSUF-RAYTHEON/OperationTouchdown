# ArUco Marker Detector
# Returns 3D position of marker in meters
#
# Assumptions:
# - Dictionary: 6x6_250 (configurable below)
# - Target marker ID: 67
# - Marker size: 20 cm (0.20 meters)
#
# Mirrors the structure of april_tag_detector.py so callers can swap
# between detectors without changing the surrounding code.

import cv2
import cv2.aruco as aruco
import numpy as np
import depthai as dai


# Config / Adjust these as we test
TARGET_MARKER_ID = 67
MARKER_SIZE      = 0.20     # 20 cm marker

# ArUco dictionary.
# DICT_6X6_250 supports marker IDs 0–249 and gives a comfortable margin
# of error against false detections at our flight ranges (1–4 m).
# Change this here if the printed marker uses a different dictionary
# (e.g. DICT_4X4_50, DICT_5X5_100, DICT_APRILTAG_36h11, ...).
ARUCO_DICT = aruco.DICT_6X6_250

# Gamma uses the photographic convention:
#   output = (pixel / 255) ^ (1 / gamma) × 255
#
#  gamma > 1.0  →  BRIGHTENS dark pixels  (shadow / under-exposure tier)
#  gamma < 1.0  →  DARKENS  bright pixels (glare / over-exposure tier)
#
# ── Shadow / under-exposure tier (gamma > 1) ────────────────────────────────
# _GAMMA_MODERATE   partial shadow, one side of the marker in shade
# _GAMMA_DEEP       marker mostly in deep shadow
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
    marker's white and black squares gets expanded to fill the full 0–255
    range before CLAHE runs on top of it.

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
    gradient, and dividing it out reveals the marker's local contrast regardless
    of whether the background is much brighter than the marker.  It also handles
    uneven shadows that fall across only part of the frame.

    blur_ksize controls how large a gradient is removed; 71 pixels at 300×300
    captures most real-world lighting gradients without erasing the marker
    itself (which is ~50–100 pixels wide).
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

    Applied AFTER normalisation passes to give the ArUco quad finder crisper,
    better-defined marker edges.  Spotty / intermittent detection is often
    caused by soft edges that the quad-finder finds on some frames but not
    others; sharpening makes the response consistent across frames.

    sigma=2.0 sharpens at the scale of a marker edge (~2 px) without creating
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
    marker pattern (~50–100 px wide).  scale=48 centers the output at 128 with
    a comfortable ±48 range for typical outdoor contrast levels.
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
    brightness: a marker sitting in pitch-black shadow looks identical to one
    in full sunlight as long as the white/black squares differ even slightly
    from their immediate surroundings.

    block_size must be odd and > 1.
    c is subtracted from the local mean before thresholding; positive values
    make the threshold stricter (fewer pixels become 'white').

    Two block sizes are used in the pipeline:
      - Small  (31)  — recovers fine marker detail in localised shadows.
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


class ArucoMarkerDetection:
    """
    Lightweight wrapper that mimics the pupil_apriltags Detection object so
    callers can use the same fields regardless of which detector is in use:

      .tag_id   — int marker ID
      .corners  — (4, 2) float32 pixel coordinates
      .center   — (2,) float32 center pixel coordinate
      .pose_t   — (3, 1) translation vector in meters (camera frame)
      .pose_R   — (3, 3) rotation matrix
    """

    def __init__(self, marker_id, corners, pose_t, pose_R):
        self.tag_id  = int(marker_id)
        self.corners = corners.astype(np.float32)
        self.center  = corners.mean(axis=0).astype(np.float32)
        self.pose_t  = pose_t
        self.pose_R  = pose_R


class ArucoMarkerDetector:

    def __init__(self, calibration_handler):
        """
        Initialize ArUco detector using calibration from the OAK-D S2.
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
        # recovers marker structure when the shadow covers most of the marker.
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

        # ArUco detector setup.  Sub-pixel corner refinement gives noticeably
        # more stable pose at 1–4 m for negligible CPU cost.
        self._aruco_dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self._aruco_params     = aruco.DetectorParameters()
        self._aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._aruco_detector   = aruco.ArucoDetector(
            self._aruco_dictionary, self._aruco_params
        )

        # Precomputed 3-D corner positions in the marker frame.
        # ArUco's detectMarkers returns image corners in the order:
        # top-left, top-right, bottom-right, bottom-left.
        half = MARKER_SIZE / 2.0
        self._marker_obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        print("[INFO] ArUco detector initialized (intrinsics will be set on first frame)")

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

        self.camera_matrix = np.array(intrinsics, dtype=np.float64)

        self.FX = self.camera_matrix[0][0]
        self.FY = self.camera_matrix[1][1]
        self.CX = self.camera_matrix[0][2]
        self.CY = self.camera_matrix[1][2]

        self.dist_coeffs = np.array(
            self.calibration_handler.getDistortionCoefficients(
                dai.CameraBoardSocket.CAM_A
            ),
            dtype=np.float64,
        )

        print(f"[INFO] Intrinsics updated for {w}x{h}")

    def _preprocess_variants(self, gray: np.ndarray) -> list:
        """
        Return preprocessed grayscale images to try in order, from least to
        most aggressive.  ArUco's own adaptive thresholding handles most
        normal lighting, so the early passes usually succeed; the expensive
        passes only run when all lighter passes have failed.

        See april_tag_detector.py for per-pass documentation — the variants
        are identical because both detectors look for high-contrast black /
        white quad patterns.
        """
        # ── Shadow tier ────────────────────────────────────────────────────

        p1 = self._clahe.apply(gray)
        p2 = _unsharp_mask(p1)
        p3 = self._clahe.apply(cv2.LUT(gray, self._gamma_lut))
        p4 = self._clahe.apply(
            cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        )

        div_norm = _division_normalize(gray)
        p5 = self._clahe_deep.apply(div_norm)
        p6 = self._clahe_deep.apply(_unsharp_mask(div_norm))

        lsdn = _local_std_normalize(gray)
        p7 = self._clahe_deep.apply(lsdn)
        p8 = self._clahe_deep.apply(_unsharp_mask(lsdn))

        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        p9 = self._clahe_deep.apply(gamma_strong)
        p10 = self._clahe_deep.apply(
            cv2.bilateralFilter(gamma_strong, d=9, sigmaColor=75, sigmaSpace=75)
        )

        gamma_mid_dark = cv2.LUT(gray, self._gamma_lut_mid_dark)
        p11 = self._clahe_deep.apply(gamma_mid_dark)
        p12 = self._clahe_deep.apply(_division_normalize(gamma_mid_dark))

        p13 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_extreme))
        p14 = self._clahe_deep.apply(_percentile_stretch(gray))

        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p15 = self._clahe_deep.apply(_percentile_stretch(denoised))

        p16 = _adaptive_thresh(gray, block_size=31)
        p17 = _adaptive_thresh(gray, block_size=71)

        # ── Mixed / dappled light tier ──────────────────────────────────────

        p18 = self._clahe_deep.apply(_local_std_normalize(denoised))

        # ── Glare / backlight / over-exposure tier ──────────────────────────

        p19 = self._clahe.apply(div_norm)

        white_mild = cv2.LUT(gray, self._gamma_lut_white_mild)
        p20 = self._clahe.apply(white_mild)
        p21 = self._clahe.apply(_unsharp_mask(white_mild))
        p22 = self._clahe_deep.apply(_division_normalize(white_mild))
        p23 = self._clahe_deep.apply(_local_std_normalize(white_mild))

        white_moderate = cv2.LUT(gray, self._gamma_lut_white_moderate)
        p24 = self._clahe_deep.apply(white_moderate)
        p25 = self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_white_strong))

        bright_denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        p26 = self._clahe_deep.apply(
            cv2.LUT(bright_denoised, self._gamma_lut_white_moderate)
        )
        p27 = _adaptive_thresh(white_moderate, block_size=31)

        return [
            p1,  p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9,
            p10, p11, p12, p13, p14, p15, p16, p17,
            p18,
            p19, p20, p21, p22, p23, p24, p25, p26, p27,
        ]

    def _estimate_pose(self, marker_corners: np.ndarray):
        """
        Recover marker pose from its 4 image corners using IPPE_SQUARE — the
        planar-square specialised solver that gives the most accurate and
        stable pose for ArUco markers.

        Since the image is undistorted upstream in _prepare_gray, we pass a
        zeroed distortion vector so solvePnP does not double-undistort.
        """
        success, rvec, tvec = cv2.solvePnP(
            self._marker_obj_points,
            marker_corners,
            self.camera_matrix,
            np.zeros(5, dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None
        pose_R, _ = cv2.Rodrigues(rvec)
        return tvec.reshape(3, 1), pose_R

    def _detect_raw(self, image: np.ndarray):
        """
        Run the ArUco detector on a preprocessed image and return an
        ArucoMarkerDetection for TARGET_MARKER_ID, or None.  Callers that
        only need pose should use the returned object's pose_t; callers
        that need corners/center for visualization use them directly.
        """
        corners, ids, _ = self._aruco_detector.detectMarkers(image)
        if ids is None:
            return None

        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            if int(marker_id) != TARGET_MARKER_ID:
                continue
            marker_corners = corners[i].reshape(4, 2)
            pose = self._estimate_pose(marker_corners)
            if pose is None:
                return None
            pose_t, pose_R = pose
            return ArucoMarkerDetection(marker_id, marker_corners, pose_t, pose_R)
        return None

    def _preprocess_and_find(self, gray: np.ndarray):
        """
        Iterate preprocessing variants in order and return the first
        ArucoMarkerDetection that succeeds, or None if all variants fail.
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
        Detect marker and return:

        (x, y, z) in meters relative to camera frame

        Camera Frame:
        x → right or left
        y → down or up
        z → forward or back

        Returns None if marker not detected.
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
        Detect marker and return the ArucoMarkerDetection object for
        TARGET_MARKER_ID, or None if the marker is not found.

        The returned object exposes:
          .tag_id   — integer marker ID
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
