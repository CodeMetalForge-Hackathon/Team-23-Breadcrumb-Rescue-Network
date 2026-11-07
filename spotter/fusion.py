"""Camera <-> thermal mapping, person confirmation by body heat, and HEAT blob finding.

Coordinates: camera (u, v) and thermal (tu, tv) are both normalized 0..1, origin top-left.
The thermal grid can be any size (MLX90640 is 24 rows x 32 cols); its shape is read from the frame.
"""
from collections import deque, namedtuple

import numpy as np

ROWS, COLS = 24, 32          # MLX90640 default, only used when no frame shape is given

HeatBlob = namedtuple("HeatBlob", "u v tu tv dt pixels")
Person = namedtuple("Person", "det decision dt")          # decision: confirmed | uncertain | rejected
Analysis = namedtuple("Analysis", "candidates persons blobs ambient")


def cam_to_thermal(u, v, cfg):
    """Camera point -> thermal point (normalized, unclamped)."""
    tu = (u - 0.5) * cfg["scale_x"] + 0.5 + cfg["offset_x"]
    tv = (v - 0.5) * cfg["scale_y"] + 0.5 + cfg["offset_y"]
    if cfg["flip_h"]:
        tu = 1 - tu
    if cfg["flip_v"]:
        tv = 1 - tv
    return tu, tv


def thermal_to_cam(tu, tv, cfg):
    """Inverse of cam_to_thermal."""
    if cfg["flip_h"]:
        tu = 1 - tu
    if cfg["flip_v"]:
        tv = 1 - tv
    u = (tu - 0.5 - cfg["offset_x"]) / cfg["scale_x"] + 0.5
    v = (tv - 0.5 - cfg["offset_y"]) / cfg["scale_y"] + 0.5
    return u, v


def cam_to_cell(u, v, cfg, shape=(ROWS, COLS)):
    """Camera point -> (col, row) on a thermal grid of the given (rows, cols), clamped."""
    rows, cols = shape
    tu, tv = cam_to_thermal(u, v, cfg)
    return min(max(tu * cols, 0.0), cols - 1e-6), min(max(tv * rows, 0.0), rows - 1e-6)


def box_to_thermal(det, cfg):
    """Detection box -> (tu0, tv0, tu1, tv1) in thermal space, sorted (flips swap the corners)."""
    a = cam_to_thermal(det.x0, det.y0, cfg)
    b = cam_to_thermal(det.x1, det.y1, cfg)
    return min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])


def inside_fraction(rect):
    """Fraction of a thermal-space rect that lies inside the thermal view [0,1]x[0,1]."""
    tu0, tv0, tu1, tv1 = rect
    area = (tu1 - tu0) * (tv1 - tv0)
    if area <= 0:
        return 0.0
    w = max(0.0, min(tu1, 1.0) - max(tu0, 0.0))
    h = max(0.0, min(tv1, 1.0) - max(tv0, 0.0))
    return w * h / area


def rect_cells(rect, shape=(ROWS, COLS)):
    """Thermal-space rect -> (c0, c1, r0, r1) slice bounds, clamped, at least one pixel."""
    rows, cols = shape
    tu0, tv0, tu1, tv1 = rect
    c0 = min(max(int(np.floor(tu0 * cols)), 0), cols - 1)
    r0 = min(max(int(np.floor(tv0 * rows)), 0), rows - 1)
    c1 = min(max(int(np.ceil(tu1 * cols)), c0 + 1), cols)
    r1 = min(max(int(np.ceil(tv1 * rows)), r0 + 1), rows)
    return c0, c1, r0, r1


def view_of(u):
    """Position in the camera view by thirds: L, C or R."""
    if u < 1 / 3:
        return "L"
    if u >= 2 / 3:
        return "R"
    return "C"


def classify_person(det, thermal, cfg):
    """-> (decision, dt). dt is None when thermal can't judge this box."""
    if thermal is None:
        return "uncertain", None
    rect = box_to_thermal(det, cfg)
    if inside_fraction(rect) < 0.5:
        return "uncertain", None
    ambient = float(np.median(thermal))
    c0, c1, r0, r1 = rect_cells(rect, thermal.shape)
    region = np.sort(thermal[r0:r1, c0:c1], axis=None)
    dt = float(np.mean(region[-5:])) - ambient
    if dt >= cfg["person_min_dt"]:
        return "confirmed", dt
    if dt >= cfg["reject_max_dt"]:
        return "uncertain", dt
    return "rejected", dt


def _components(mask):
    """4-connected components of a boolean grid -> list of (rows, cols) index arrays."""
    seen = np.zeros_like(mask, dtype=bool)
    out = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        queue, rows, cols = deque([(r0, c0)]), [], []
        while queue:
            r, c = queue.popleft()
            rows.append(r)
            cols.append(c)
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= nr < mask.shape[0] and 0 <= nc < mask.shape[1] and mask[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True
                    queue.append((nr, nc))
        out.append((np.array(rows), np.array(cols)))
    return out


def find_heat_blobs(thermal, cfg, person_rects=()):
    """Warm regions (>= heat_min_dt above ambient, >= heat_min_pixels) whose centre is not in a person box."""
    ambient = float(np.median(thermal))
    blobs = []
    for rows, cols in _components(thermal - ambient >= cfg["heat_min_dt"]):
        if len(rows) < cfg["heat_min_pixels"]:
            continue
        tu, tv = (cols.mean() + 0.5) / thermal.shape[1], (rows.mean() + 0.5) / thermal.shape[0]
        if any(r[0] <= tu <= r[2] and r[1] <= tv <= r[3] for r in person_rects):
            continue
        u, v = thermal_to_cam(tu, tv, cfg)
        blobs.append(HeatBlob(u, v, tu, tv, float(thermal[rows, cols].max()) - ambient, len(rows)))
    return blobs


def analyze(dets, thermal, cfg, recent_dets=()):
    """One detection frame -> alert candidates (dicts with t, c, th, v, dt) + details for the debug view.
    recent_dets: person boxes from the last ~1 s. Heat inside them is not HEAT, because the 4 Hz thermal
    frame lags the camera and the detector drops frames, which would turn body heat into false HEAT alerts."""
    persons, candidates = [], []
    for det in dets:
        decision, dt = classify_person(det, thermal, cfg)
        persons.append(Person(det, decision, dt))
        if decision == "rejected":
            continue
        candidates.append({
            "t": "PERSON", "c": round(det.score * 100), "th": 1 if decision == "confirmed" else 0,
            "v": view_of((det.x0 + det.x1) / 2), "dt": None if dt is None else round(dt),
        })

    blobs, ambient = [], None
    if thermal is not None:
        ambient = float(np.median(thermal))
        blobs = find_heat_blobs(thermal, cfg, [box_to_thermal(d, cfg) for d in (*dets, *recent_dets)])
        for b in blobs:
            candidates.append({"t": "HEAT", "c": cfg["heat_conf"], "v": view_of(b.u), "dt": round(b.dt)})
    return Analysis(candidates, persons, blobs, ambient)
