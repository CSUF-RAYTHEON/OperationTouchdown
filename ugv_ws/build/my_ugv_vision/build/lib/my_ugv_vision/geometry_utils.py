"""Pure-Python geometry helpers used by the object_localizer node.

No ROS imports beyond LaserScan typing at runtime; functions accept duck-typed
objects so they can also be unit-tested with simple stand-ins.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def median_depth(
    depth_img_uint16: np.ndarray,
    cx: float,
    cy: float,
    win: int,
    min_m: float,
    max_m: float,
) -> float:
    """Return the median depth in meters within a ``win`` x ``win`` window
    around (cx, cy). ``depth_img_uint16`` is millimeters (OAK-D convention).
    Returns NaN if no valid samples.
    """
    if depth_img_uint16 is None or depth_img_uint16.size == 0:
        return float('nan')
    h, w = depth_img_uint16.shape[:2]
    half = max(1, int(win)) // 2
    icx = int(round(cx))
    icy = int(round(cy))
    x0 = max(0, icx - half)
    x1 = min(w, icx + half + 1)
    y0 = max(0, icy - half)
    y1 = min(h, icy + half + 1)
    if x1 <= x0 or y1 <= y0:
        return float('nan')
    patch = depth_img_uint16[y0:y1, x0:x1].astype(np.float32) * 0.001
    valid = patch[(patch > min_m) & (patch < max_m)]
    if valid.size == 0:
        return float('nan')
    return float(np.median(valid))


def bearing_from_pixel(cx: float, fx: float, cx_pp: float) -> float:
    """Horizontal bearing (radians) of a pixel column ``cx`` given fx and
    principal-point x ``cx_pp``. Positive bearing = right of optical axis
    in the camera's optical frame.
    """
    return math.atan2(cx - cx_pp, fx)


def min_range_in_window(
    scan,
    theta_center: float,
    theta_window: float,
) -> float:
    """Smallest finite range in the scan within
    [theta_center - theta_window, theta_center + theta_window].

    ``scan`` is a ``sensor_msgs/LaserScan``-like duck object with
    ``angle_min``, ``angle_increment``, ``range_min``, ``range_max``, and
    ``ranges`` attributes. Returns NaN if no valid range.
    """
    if scan is None or not getattr(scan, 'ranges', None):
        return float('nan')

    ranges = np.asarray(scan.ranges, dtype=np.float32)
    n = ranges.shape[0]
    if n == 0:
        return float('nan')

    inc = float(scan.angle_increment)
    if inc == 0.0:
        return float('nan')
    angle_min = float(scan.angle_min)

    lo = theta_center - theta_window
    hi = theta_center + theta_window
    i_lo = int(math.floor((lo - angle_min) / inc))
    i_hi = int(math.ceil((hi - angle_min) / inc))
    i_lo = max(0, min(n - 1, i_lo))
    i_hi = max(0, min(n - 1, i_hi))
    if i_hi < i_lo:
        i_lo, i_hi = i_hi, i_lo

    window = ranges[i_lo:i_hi + 1]
    rmin = float(getattr(scan, 'range_min', 0.0))
    rmax = float(getattr(scan, 'range_max', math.inf))
    mask = np.isfinite(window) & (window >= rmin) & (window <= rmax)
    if not np.any(mask):
        return float('nan')
    return float(np.min(window[mask]))


def pixel_to_3d(
    pinhole_cam,
    cx: float,
    cy: float,
    z: float,
) -> Tuple[float, float, float]:
    """Back-project a pixel through the pinhole camera at depth z (meters).

    Returns the 3D point in the camera optical frame.
    """
    ray = pinhole_cam.projectPixelTo3dRay((float(cx), float(cy)))
    if ray[2] == 0.0:
        return (float('nan'), float('nan'), float('nan'))
    scale = z / float(ray[2])
    return (float(ray[0] * scale), float(ray[1] * scale), float(z))
