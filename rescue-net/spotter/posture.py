"""Upright or on the ground, from YOLOv8-pose keypoints (COCO order) with a box-shape fallback.

A person lying on the floor is the likeliest casualty, so PERSON alerts carry ps = "D" (down) or "U" (upright).
Coordinates are normalized 0..1 on a 16:9 main stream; x is rescaled to pixels before measuring angles."""
import math

L_SH, R_SH, L_HIP, R_HIP = 5, 6, 11, 12
KPT_MIN_CONF = 0.5


def _mid(kpts, a, b):
    pts = [kpts[i] for i in (a, b) if kpts[i][2] >= KPT_MIN_CONF]
    if not pts:
        return None
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def torso_angle(kpts, x_scale):
    """Angle of the shoulders->hips line from vertical, in degrees (0 = standing), or None if not visible."""
    sh, hp = _mid(kpts, L_SH, R_SH), _mid(kpts, L_HIP, R_HIP)
    if sh is None or hp is None:
        return None
    dx, dy = (hp[0] - sh[0]) * x_scale, hp[1] - sh[1]
    if math.hypot(dx, dy) < 0.03:             # torso too small or keypoints collapsed: don't trust it
        return None
    return math.degrees(math.atan2(abs(dx), abs(dy)))


def posture(det, cfg, x_scale=16 / 9):
    """-> "D" (on the ground), "U" (upright) or None (can't tell)."""
    kpts = getattr(det, "kpts", None)
    if kpts is not None:
        ang = torso_angle(kpts, x_scale)
        if ang is not None:
            return "D" if ang >= cfg["down_angle_deg"] else "U"
    w, h = (det.x1 - det.x0) * x_scale, det.y1 - det.y0
    if h <= 0:
        return None
    ratio = w / h
    if ratio >= cfg["down_aspect"]:
        return "D"
    return "U" if ratio <= 0.8 else None
