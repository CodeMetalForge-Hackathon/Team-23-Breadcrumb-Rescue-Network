"""Where a detected person is, in the LiDAR frame (same convention as spotter/lidar.py):
bearing in degrees clockwise from LiDAR 0, point = (sin(b) * r, cos(b) * r).

Bearing comes from the box centre, the camera's horizontal field of view and its yaw (the LiDAR angle the
camera's optical axis points at). Range is the LiDAR's nearest surface within +-3 deg of that bearing; when the
LiDAR has nothing there, or only something far behind (a person lying on the floor is below the scan plane),
range comes from the box size instead ("rs": "S").
"""
import math
from collections import namedtuple

import numpy as np

from spotter.posture import classify

# src "C" = Pi camera, "T" = USB thermal camera. b in deg, r in m (or None), rs "L" (LiDAR) / "S" (box size),
# ps "U" / "D" / "?", why = what the posture is based on (debug view).
Obs = namedtuple("Obs", "src b r rs ps c det u why", defaults=("",))

# per camera: config keys for field of view, yaw, direction (+1: image right = clockwise), and image h / w
CAMERAS = {
    "C": ("cam_hfov", "cam_yaw", "cam_dir", 9 / 16),      # Camera Module 3 (IMX708), 16:9 (1536x864 mode)
    "T": ("th_hfov", "th_yaw", "th_dir", 512 / 640),      # USB thermal camera, 640x512
}


def angdiff(a, b):
    """a - b wrapped to -180..180 (works on numpy arrays too)."""
    return (a - b + 180.0) % 360.0 - 180.0


def bearing(u, hfov, yaw, direction=1):
    """Box centre u (0..1 across the image) -> LiDAR bearing in degrees (pinhole camera)."""
    off = math.degrees(math.atan((u - 0.5) * 2 * math.tan(math.radians(hfov) / 2)))
    return (yaw + direction * off) % 360.0


def lidar_range(scan, b, win=3.0, min_m=0.25, depth_m=0.5):
    """Median of the nearest group of LiDAR returns within +-win deg of bearing b, in metres, or None.
    Returns under min_m are the rig itself. Only returns within depth_m of the nearest one count, so a wall
    behind a thin person does not pull the range back."""
    if scan is None:
        return None
    a, d = scan
    a, d = np.asarray(a, np.float32), np.asarray(d, np.float32) / 1000.0
    m = (np.abs(angdiff(a, b)) <= win) & (d >= min_m)
    if m.sum() < 2:
        return None
    dd = np.sort(d[m])
    return float(np.median(dd[dd <= dd[0] + depth_m]))


def size_range(det, hfov, aspect_hw, ps, height_m=1.7, length_m=1.6):
    """Rough range from the box: body height (upright) or length (on the ground) over its angular size."""
    tx = math.tan(math.radians(hfov) / 2)
    w = (det.x1 - det.x0) * 2 * tx                 # extent on an image plane 1 m away
    h = (det.y1 - det.y0) * 2 * tx * aspect_hw
    ext, size = (max(w, h), length_m) if ps == "D" else (h, height_m)
    return None if ext < 0.02 else size / ext


def observe(det, src, scan, cfg, x_scale=None, roll_deg=0.0):
    """One detection -> Obs in the LiDAR frame, with posture "U" / "D" / "?" (see spotter/posture.py)."""
    hfov_k, yaw_k, dir_k, aspect_hw = CAMERAS[src]
    hfov, yaw, direction = float(cfg[hfov_k]), float(cfg[yaw_k]), int(cfg[dir_k])
    u = (det.x0 + det.x1) / 2
    b = bearing(u, hfov, yaw, direction)
    r_l = lidar_range(scan, b, cfg["lidar_win_deg"], cfg["lidar_min_m"])
    r_down = size_range(det, hfov, aspect_hw, "D")
    # the LiDAR return is this body (it crosses the scan plane), not the wall behind someone lying below it
    body = r_l is not None and r_down is not None and r_l <= 1.4 * r_down + 0.3
    scale = 2 * math.tan(math.radians(hfov) / 2) * aspect_hw * r_l if body else None   # metres per image height
    extent = (det.y1 - det.y0) * scale if body else None
    # lidar_body_veto off when the rig sits on the floor: its scan plane then hits people lying down too
    ps, why = classify(det, cfg, x_scale or 1 / aspect_hw, roll_deg, extent, body and cfg["lidar_body_veto"],
                       scale)
    r_s = r_down if ps == "D" else size_range(det, hfov, aspect_hw, "U")
    # LiDAR far behind what the box size says: it is looking past the person (lying below the scan plane)
    if r_l is not None and (r_s is None or r_l <= 1.8 * r_s + 0.5):
        r, rs = r_l, "L"
    else:
        r, rs = r_s, "S" if r_s is not None else None
    return Obs(src, b, r, rs, ps, round(det.score * 100), det, u, why)
