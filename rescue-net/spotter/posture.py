"""Upright ("U"), on the ground ("D") or can't tell ("?"), from YOLOv8-pose keypoints (COCO order), the box,
and (via spotter.locate) the LiDAR.

A false "person down / possible casualty" is worse than "?", so D needs positive evidence:
  - a body segment far from vertical: the torso (shoulders -> hips), else the shins (knees -> ankles), else
    hips -> ankles; >= down_angle_deg is D, <= up_angle_deg is U, in between is "?". The rig's roll (from the
    accelerometer, when imu_roll_axis is set) is taken out first;
  - and nothing contradicting it: the LiDAR hitting the body at the person's range means the body crosses the
    scan plane (chest / waist height on the rig), so the person is not lying flat; a visible vertical extent of
    upright_min_m (0.9 m) or more cannot be a person lying down (a box cut by the image edge still gives a lower
    bound, so this also turns "?" into U).
The box shape never gives D: the camera's field of view is narrow, and a box cut by the image edge (or a person
partly hidden) says nothing about posture. An uncut box clearly taller than wide gives U.
Coordinates are normalised 0..1; x_scale = image width / height, so x * x_scale is comparable to y.
"""
import math

L_SH, R_SH, L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANK, R_ANK = 5, 6, 11, 12, 13, 14, 15, 16
KPT_MIN_CONF = 0.5
SEGMENTS = (("torso", (L_SH, R_SH), (L_HIP, R_HIP), 0.04),
            ("shins", (L_KNEE, R_KNEE), (L_ANK, R_ANK), 0.04),
            ("legs", (L_HIP, R_HIP), (L_ANK, R_ANK), 0.06))


def _mid(kpts, a, b):
    pts = [kpts[i] for i in (a, b) if kpts[i][2] >= KPT_MIN_CONF]
    if not pts:
        return None
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def segment_angle(kpts, top, bottom, x_scale, roll_deg=0.0, min_len=0.04):
    """Angle of the top -> bottom line from vertical, in degrees (0 = upright), or None if not visible."""
    p, q = _mid(kpts, *top), _mid(kpts, *bottom)
    if p is None or q is None:
        return None
    dx, dy = (q[0] - p[0]) * x_scale, q[1] - p[1]
    if math.hypot(dx, dy) < min_len:                 # too short, or keypoints collapsed: don't trust it
        return None
    if roll_deg:                                     # rig rolled clockwise (seen from behind): undo it
        c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        dx, dy = c * dx - s * dy, s * dx + c * dy
    return math.degrees(math.atan2(abs(dx), abs(dy)))


def torso_angle(kpts, x_scale, roll_deg=0.0):
    """Shoulders -> hips angle from vertical (kept for the debug view and older callers)."""
    return segment_angle(kpts, (L_SH, R_SH), (L_HIP, R_HIP), x_scale, roll_deg)


def _get(cfg, key, default):
    """cfg may be a dict or a spotter.config.Config (which only has [])."""
    try:
        return cfg[key]
    except KeyError:
        return default


def cut_by_edge(det, margin=0.025):
    return det.x0 <= margin or det.y0 <= margin or det.x1 >= 1 - margin or det.y1 >= 1 - margin


def classify(det, cfg, x_scale=16 / 9, roll_deg=0.0, extent_m=None, lidar_body=False):
    """-> (ps, why): ps is "U", "D" or "?". extent_m = visible height in metres (LiDAR range known),
    lidar_body = the LiDAR has a return from this person's body."""
    up_a, down_a = _get(cfg, "up_angle_deg", 35), _get(cfg, "down_angle_deg", 60)
    ps, why, ang = "?", "no body segment visible", None
    kpts = getattr(det, "kpts", None)
    if kpts is not None:
        for name, top, bottom, min_len in SEGMENTS:
            ang = segment_angle(kpts, top, bottom, x_scale, roll_deg, min_len)
            if ang is not None:
                ps = "D" if ang >= down_a else "U" if ang <= up_a else "?"
                why = f"{name} {ang:.0f}deg"
                break
    if ang is None and not cut_by_edge(det):
        w, h = (det.x1 - det.x0) * x_scale, det.y1 - det.y0
        if h > 0 and w / h <= 0.8:
            ps, why = "U", f"box {w / h:.2f} wide/high"
    tall = extent_m is not None and extent_m >= _get(cfg, "upright_min_m", 0.9)
    # a clearly horizontal body fully in view stands (lies) on its own; the LiDAR / extent checks are for the
    # doubtful cases (cut by the image edge, or an angle near the threshold), so a wall right behind a person
    # lying on the floor does not hide them
    doubtful = cut_by_edge(det) or ang is None or ang < _get(cfg, "down_sure_deg", 75)
    if ps == "D" and tall and doubtful:
        return "?", why + f", but {extent_m:.1f} m tall"
    if ps == "D" and lidar_body and doubtful:
        return "?", why + ", but the LiDAR hits the body"
    if ps == "?" and tall:
        return "U", why + f", {extent_m:.1f} m tall"
    return ps, why


def posture(det, cfg, x_scale=16 / 9, roll_deg=0.0):
    """-> "D", "U" or None (can't tell), from the image alone (no LiDAR evidence)."""
    ps, _ = classify(det, cfg, x_scale, roll_deg)
    return None if ps == "?" else ps
