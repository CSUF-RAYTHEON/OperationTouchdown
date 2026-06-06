# Nested AprilTag (36h11) Detector  —  cv2.aruco backend
# Returns 3D position of BOTH layers of a nested board in meters.
#
# A nested board is a large OUTER 36h11 tag with a small INNER 36h11 tag
# printed inside its white center (see the Nested_AprilTags generator):
#
#     +-----------------------------+
#     |  OUTER tag  (long range)    |
#     |        +---------+          |
#     |        | INNER   |          |
#     |        |  tag    |          |
#     |        +---------+          |
#     |                             |
#     +-----------------------------+
#
# The OUTER tag is acquired first (visible from far away); the INNER tag
# takes over for the final close-range precision step, because it stays
# resolvable when the OUTER tag overflows the field of view.
#
# Why a plain single-pass detector is not enough:
#   The INNER tag sits inside the OUTER tag's data area, so a single
#   detectMarkers() pass decodes the INNER cleanly but corrupts the OUTER
#   payload.  We therefore run two passes per image: detect, mask out every
#   tag we found, then detect again so the OUTER payload can be read once the
#   INNER tag is painted over.  (Ported from Nested_AprilTags/tracker_engine.py)
#
# This module intentionally does NOT import depthai, so it can run both on the
# drone (intrinsics supplied from the OAK-D calibration) and off-drone on a
# laptop webcam or on the generated PNG files (no intrinsics -> corners only).

import cv2
import cv2.aruco as aruco
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Config / Adjust these as we test
# ─────────────────────────────────────────────────────────────────────────────

# Tag IDs.  INNER is fixed at 12 by the board generator.  OUTER varies per
# board (file name board_outer_<OUTER>_inner_12.png) — set it to the board you
# actually printed.
OUTER_TAG_ID = 77      # outer tag ID of the printed board (configurable)
INNER_TAG_ID = 12      # inner tag ID — fixed by generate_board_nest.py

# Physical PRINTED size of each tag, in METERS — measure your printout!
# This is the side length of the tag's black square (the square ArUco locks
# its 4 corners onto), NOT including the surrounding white quiet zone.
#
# On a generated board the OUTER black square spans the full 640 px core and
# the INNER black square spans the central 160 px, so geometrically:
#       INNER_TAG_SIZE  ==  OUTER_TAG_SIZE / 4
# If you measured them independently with a ruler, just type both in directly.
#
# !!! EDIT THESE TWO NUMBERS to match your physical printout !!!
OUTER_TAG_SIZE = 0.34            # meters — outer black square side length (measured)
INNER_TAG_SIZE = 0.09            # meters — inner black square side length (measured)

# Both tags are AprilTag 36h11 — the same family the generator uses.
ARUCO_DICT = aruco.DICT_APRILTAG_36h11

# Temporal cache: bridge brief dropouts so the box does not flicker.
# Set to 0 to disable.  (Ported from tracker_engine.TagTrackerCache.)
MAX_MISSING_FRAMES = 4

# Gamma uses the photographic convention:
#   output = (pixel / 255) ^ (1 / gamma) × 255
#  gamma > 1.0  →  BRIGHTENS dark pixels  (shadow / under-exposure tier)
#  gamma < 1.0  →  DARKENS  bright pixels (glare / over-exposure tier)
_GAMMA_MODERATE = 2.2
_GAMMA_DEEP     = 4.0
_GAMMA_MID_DARK = 6.0
_GAMMA_EXTREME  = 8.0
_GAMMA_WHITE_MILD     = 0.5
_GAMMA_WHITE_MODERATE = 0.3
_GAMMA_WHITE_STRONG   = 0.15

# Percentile clipping for the histogram-stretch passes.
_STRETCH_LOW_PCT  = 1.0
_STRETCH_HIGH_PCT = 99.0


# ─────────────────────────────────────────────────────────────────────────────
# Preprocessing helpers (identical pipeline to aruco_marker_detector.py)
# ─────────────────────────────────────────────────────────────────────────────

def _build_gamma_lut(gamma: float) -> np.ndarray:
    inv = 1.0 / gamma
    table = np.array(
        [min(int((i / 255.0) ** inv * 255.0 + 0.5), 255) for i in range(256)],
        dtype=np.uint8,
    )
    return table


def _percentile_stretch(img: np.ndarray) -> np.ndarray:
    lo = np.percentile(img, _STRETCH_LOW_PCT)
    hi = np.percentile(img, _STRETCH_HIGH_PCT)
    if hi <= lo:
        return img
    stretched = np.clip(
        (img.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255
    ).astype(np.uint8)
    return stretched


def _division_normalize(img: np.ndarray, blur_ksize: int = 71) -> np.ndarray:
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
    blurred = cv2.GaussianBlur(img, (0, 0), sigmaX=sigma)
    return cv2.addWeighted(img, 1.0 + strength, blurred, -strength, 0)


def _local_std_normalize(
    img: np.ndarray, ksize: int = 31, scale: float = 48.0
) -> np.ndarray:
    img_f = img.astype(np.float32)
    k = (ksize, ksize)
    local_mean   = cv2.GaussianBlur(img_f, k, 0)
    local_sq_mean = cv2.GaussianBlur(img_f * img_f, k, 0)
    variance     = local_sq_mean - local_mean * local_mean
    local_std    = np.sqrt(np.maximum(variance, 0.0)) + 1.0
    normalized   = (img_f - local_mean) / local_std * scale + 128.0
    return np.clip(normalized, 0, 255).astype(np.uint8)


def _adaptive_thresh(img: np.ndarray, block_size: int, c: int = 5) -> np.ndarray:
    if block_size % 2 == 0:
        block_size += 1
    return cv2.adaptiveThreshold(
        img, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        block_size,
        c,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Detection result object  (mirrors pupil_apriltags / ArucoMarkerDetection)
# ─────────────────────────────────────────────────────────────────────────────

class NestedTagDetection:
    """
    One detected tag layer.  Fields match the other detectors so existing
    callers can use it unchanged:

      .tag_id   — int tag ID
      .corners  — (4, 2) float32 pixel coordinates (TL, TR, BR, BL)
      .center   — (2,) float32 center pixel coordinate
      .pose_t   — (3, 1) translation vector in meters, or None if no intrinsics
      .pose_R   — (3, 3) rotation matrix, or None if no intrinsics
      .layer    — "outer" or "inner"
    """

    def __init__(self, tag_id, corners, pose_t, pose_R, layer):
        self.tag_id  = int(tag_id)
        self.corners = corners.astype(np.float32)
        self.center  = corners.mean(axis=0).astype(np.float32)
        self.pose_t  = pose_t
        self.pose_R  = pose_R
        self.layer   = layer


class _TagTrackerCache:
    """Frame-to-frame memory that briefly retains a tag after it is lost, so
    the overlay does not flicker.  Ported from tracker_engine.TagTrackerCache."""

    def __init__(self, max_missing_frames=4):
        self.cached = {}   # id -> (corners, missing_counter)
        self.max_missing = max_missing_frames

    def update(self, current):
        """current: dict {id: corners}.  Returns dict {id: corners} including
        recently-seen tags that dropped out for <= max_missing frames."""
        result = dict(current)
        for tid in current:
            self.cached[tid] = (current[tid], 0)
        for tid in list(self.cached.keys()):
            if tid not in current:
                corners, missing = self.cached[tid]
                if missing < self.max_missing:
                    self.cached[tid] = (corners, missing + 1)
                    result[tid] = corners
                else:
                    del self.cached[tid]
        return result


# ─────────────────────────────────────────────────────────────────────────────
# Nested detector
# ─────────────────────────────────────────────────────────────────────────────

class NestedArucoDetector:

    def __init__(
        self,
        camera_matrix=None,
        dist_coeffs=None,
        outer_id: int = OUTER_TAG_ID,
        inner_id: int = INNER_TAG_ID,
        outer_size: float = OUTER_TAG_SIZE,
        inner_size: float = INNER_TAG_SIZE,
        dict_id: int = ARUCO_DICT,
        max_missing_frames: int = MAX_MISSING_FRAMES,
    ):
        """
        camera_matrix / dist_coeffs : optional camera intrinsics.
            * Supply them (e.g. from the OAK-D calibration) to get full 3-D
              pose for each layer.
            * Leave them None (laptop webcam / PNG testing) to get detections
              with corners + center only; pose_t / pose_R will be None.
            They can also be set later with set_intrinsics().

        outer_id / inner_id   : the two tag IDs of the nested board.
        outer_size / inner_size : physical printed side length of each tag's
            black square, in meters — used for per-layer pose.
        """
        self.camera_matrix = (
            np.asarray(camera_matrix, dtype=np.float64)
            if camera_matrix is not None else None
        )
        self.dist_coeffs = (
            np.asarray(dist_coeffs, dtype=np.float64)
            if dist_coeffs is not None else None
        )

        self.outer_id = int(outer_id)
        self.inner_id = int(inner_id)
        self._obj_points = {
            self.outer_id: self._make_obj_points(outer_size),
            self.inner_id: self._make_obj_points(inner_size),
        }

        # CLAHE objects shared by the preprocessing passes.
        self._clahe      = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self._clahe_deep = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))

        self._gamma_lut          = _build_gamma_lut(_GAMMA_MODERATE)
        self._gamma_lut_strong   = _build_gamma_lut(_GAMMA_DEEP)
        self._gamma_lut_mid_dark = _build_gamma_lut(_GAMMA_MID_DARK)
        self._gamma_lut_extreme  = _build_gamma_lut(_GAMMA_EXTREME)
        self._gamma_lut_white_mild     = _build_gamma_lut(_GAMMA_WHITE_MILD)
        self._gamma_lut_white_moderate = _build_gamma_lut(_GAMMA_WHITE_MODERATE)
        self._gamma_lut_white_strong   = _build_gamma_lut(_GAMMA_WHITE_STRONG)

        # ArUco detector with sub-pixel corner refinement.
        self._dictionary = aruco.getPredefinedDictionary(dict_id)
        self._params     = aruco.DetectorParameters()
        self._params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._detector   = aruco.ArucoDetector(self._dictionary, self._params)

        self._cache = (
            _TagTrackerCache(max_missing_frames) if max_missing_frames > 0 else None
        )

        print(
            f"[INFO] Nested ArUco detector ready "
            f"(outer={self.outer_id}, inner={self.inner_id}, "
            f"pose={'on' if self.camera_matrix is not None else 'off (no intrinsics)'})"
        )

    # ── intrinsics ──────────────────────────────────────────────────────────

    @staticmethod
    def _make_obj_points(size: float) -> np.ndarray:
        """3-D corners of a tag's black square in its own frame, matching the
        ArUco corner order TL, TR, BR, BL."""
        half = size / 2.0
        return np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

    def set_intrinsics(self, camera_matrix, dist_coeffs):
        """Set / update camera intrinsics so pose can be estimated."""
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs   = np.asarray(dist_coeffs, dtype=np.float64)

    # ── preprocessing ─────────────────────────────────────────────────────────

    def _preprocess_variants(self, gray: np.ndarray):
        """Lazy generator of the 27 lighting-normalized variants, least to most
        aggressive.  See april_tag_detector.py for per-pass documentation.

        This is a GENERATOR (not a list) on purpose: callers break out the
        instant a target is found, so the expensive later transforms (bilateral
        filters, big-kernel blurs, division / local-std normalisation) are only
        computed when the earlier passes fail.  The old list form computed all
        27 up front on every frame, which dominated detection latency at large
        resolutions.  Pass order / contents are unchanged."""
        p1 = self._clahe.apply(gray)
        yield p1
        yield _unsharp_mask(p1)
        yield self._clahe.apply(cv2.LUT(gray, self._gamma_lut))
        yield self._clahe.apply(
            cv2.bilateralFilter(gray, d=9, sigmaColor=75, sigmaSpace=75)
        )

        div_norm = _division_normalize(gray)
        yield self._clahe_deep.apply(div_norm)
        yield self._clahe_deep.apply(_unsharp_mask(div_norm))

        lsdn = _local_std_normalize(gray)
        yield self._clahe_deep.apply(lsdn)
        yield self._clahe_deep.apply(_unsharp_mask(lsdn))

        gamma_strong = cv2.LUT(gray, self._gamma_lut_strong)
        yield self._clahe_deep.apply(gamma_strong)
        yield self._clahe_deep.apply(
            cv2.bilateralFilter(gamma_strong, d=9, sigmaColor=75, sigmaSpace=75)
        )

        gamma_mid_dark = cv2.LUT(gray, self._gamma_lut_mid_dark)
        yield self._clahe_deep.apply(gamma_mid_dark)
        yield self._clahe_deep.apply(_division_normalize(gamma_mid_dark))

        yield self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_extreme))
        yield self._clahe_deep.apply(_percentile_stretch(gray))

        denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        yield self._clahe_deep.apply(_percentile_stretch(denoised))

        yield _adaptive_thresh(gray, block_size=31)
        yield _adaptive_thresh(gray, block_size=71)

        yield self._clahe_deep.apply(_local_std_normalize(denoised))

        yield self._clahe.apply(div_norm)

        white_mild = cv2.LUT(gray, self._gamma_lut_white_mild)
        yield self._clahe.apply(white_mild)
        yield self._clahe.apply(_unsharp_mask(white_mild))
        yield self._clahe_deep.apply(_division_normalize(white_mild))
        yield self._clahe_deep.apply(_local_std_normalize(white_mild))

        white_moderate = cv2.LUT(gray, self._gamma_lut_white_moderate)
        yield self._clahe_deep.apply(white_moderate)
        yield self._clahe_deep.apply(cv2.LUT(gray, self._gamma_lut_white_strong))

        bright_denoised = cv2.GaussianBlur(gray, (5, 5), 0)
        yield self._clahe_deep.apply(
            cv2.LUT(bright_denoised, self._gamma_lut_white_moderate)
        )
        yield _adaptive_thresh(white_moderate, block_size=31)

    # ── two-pass nested detection ─────────────────────────────────────────────

    def _detect_two_pass(self, image: np.ndarray, found: dict):
        """
        Run the nested two-pass detection on one preprocessed image and add any
        newly found target corners (keyed by tag id) into `found`.

        Pass 1: detect markers directly — usually catches the INNER tag.
        Pass 2: paint over every detected quad with the local background color,
                then detect again so the OUTER tag's payload can be decoded once
                the INNER tag is masked out.
        """
        corners1, ids1, _ = self._detector.detectMarkers(image)

        if ids1 is not None:
            for i, tid in enumerate(ids1.flatten()):
                tid = int(tid)
                if tid in self._obj_points and tid not in found:
                    found[tid] = corners1[i].reshape(4, 2)

        # Pass 2 only helps if we already saw something to mask out and we are
        # still missing a target layer.
        if ids1 is None or self._has_all_targets(found):
            return

        h, w = image.shape[:2]
        masked = image.copy()
        for c in corners1:
            pts = c.reshape((-1, 2)).astype(np.float32)
            center = pts.mean(axis=0)

            # Sample the quiet-zone color from a thin ring just OUTSIDE the
            # tag — the white isolation border the generator prints around the
            # inner tag.  Filling the inner tag with this color (rather than
            # the median of the tag itself, which is ~mid-gray) reconstructs
            # the white center the OUTER payload expects, so the OUTER tag
            # decodes on the second pass.  Sampling the ring (instead of
            # hard-coding white) keeps this working under real-world lighting.
            ring_outer = (center + 1.35 * (pts - center)).astype(np.int32)
            ring_inner = (center + 1.10 * (pts - center)).astype(np.int32)
            ring_mask = np.zeros((h, w), dtype=np.uint8)
            cv2.fillPoly(ring_mask, [ring_outer], 255)
            cv2.fillPoly(ring_mask, [ring_inner], 0)
            ring_vals = image[ring_mask == 255]
            fill_val = int(np.median(ring_vals)) if ring_vals.size else 255

            # Paint over the tag plus a small margin to eliminate edge bleed.
            fill_poly = (center + 1.25 * (pts - center)).astype(np.int32)
            cv2.fillPoly(masked, [fill_poly], color=fill_val)

        corners2, ids2, _ = self._detector.detectMarkers(masked)
        if ids2 is not None:
            for i, tid in enumerate(ids2.flatten()):
                tid = int(tid)
                if tid in self._obj_points and tid not in found:
                    found[tid] = corners2[i].reshape(4, 2)

    def _has_all_targets(self, found: dict) -> bool:
        return self.outer_id in found and self.inner_id in found

    def _landing_target_ready(self, found: dict) -> bool:
        return self.inner_id in found or self.outer_id in found

    def _select_landing_corners(self, found: dict):
        if self.inner_id in found:
            return self.inner_id, found[self.inner_id]
        if self.outer_id in found:
            return self.outer_id, found[self.outer_id]
        return None, None

    def _detection_from_corners(self, tag_id: int, corners: np.ndarray):
        layer = "outer" if tag_id == self.outer_id else "inner"
        pose = self._estimate_pose(tag_id, corners)
        if pose is None:
            return None
        pose_t, pose_R = pose
        return NestedTagDetection(tag_id, corners, pose_t, pose_R, layer)

    def _estimate_pose(self, tag_id: int, corners: np.ndarray):
        """Per-layer pose via IPPE_SQUARE.  Returns (pose_t, pose_R) or None.
        Returns None when no intrinsics are configured."""
        if self.camera_matrix is None:
            return None
        success, rvec, tvec = cv2.solvePnP(
            self._obj_points[tag_id],
            corners.astype(np.float32),
            self.camera_matrix,
            np.zeros(5, dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None
        pose_R, _ = cv2.Rodrigues(rvec)
        return tvec.reshape(3, 1), pose_R

    def _prepare_gray(self, frame: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            gray = cv2.undistort(gray, self.camera_matrix, self.dist_coeffs)
        return gray

    # ── public API ────────────────────────────────────────────────────────────

    def detect(self, frame):
        """
        Detect both layers of the nested board.

        Returns a list of NestedTagDetection (0, 1, or 2 items): the OUTER tag,
        the INNER tag, or both, depending on what is currently visible.
        """
        if frame is None:
            return []

        gray = self._prepare_gray(frame)

        found = {}
        for variant in self._preprocess_variants(gray):
            self._detect_two_pass(variant, found)
            if self.inner_id in found:
                break
            if self._has_all_targets(found):
                break

        if self._cache is not None:
            found = self._cache.update(found)

        results = []
        for tag_id, corners in found.items():
            det = self._detection_from_corners(tag_id, corners)
            if det is not None:
                results.append(det)
        return results

    def get_layer(self, frame, layer: str):
        """Return the NestedTagDetection for "outer" or "inner", or None."""
        for det in self.detect(frame):
            if det.layer == layer:
                return det
        return None

    def get_landing_target(self, frame):
        """
        Convenience policy for precision landing: prefer the INNER tag when it
        is visible (best for the final close-range step), otherwise fall back
        to the OUTER tag (long-range acquisition).

        Returns a single NestedTagDetection or None.
        """
        if frame is None:
            return None

        gray = self._prepare_gray(frame)
        found = {}

        def _try_landing_target():
            merged = self._cache.update(dict(found)) if self._cache else found
            tag_id, corners = self._select_landing_corners(merged)
            if tag_id is None:
                return None
            return self._detection_from_corners(tag_id, corners)

        self._detect_two_pass(gray, found)
        det = _try_landing_target()
        if det is not None:
            return det

        for variant in self._preprocess_variants(gray):
            self._detect_two_pass(variant, found)
            det = _try_landing_target()
            if det is not None:
                return det
            if self.inner_id in found:
                break
            if self._landing_target_ready(found):
                break

        return None

    def get_tag_pose(self, frame):
        """
        Drop-in match for the other detectors: return (x, y, z) in meters for
        the current landing-target layer, or None.  Requires intrinsics.
        """
        det = self.get_landing_target(frame)
        if det is None or det.pose_t is None:
            return None
        t = det.pose_t
        return float(t[0][0]), float(t[1][0]), float(t[2][0])
