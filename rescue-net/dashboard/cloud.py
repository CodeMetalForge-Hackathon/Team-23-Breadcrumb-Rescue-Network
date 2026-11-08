"""Bounded, tilt-relative LiDAR cloud for the offline command-post view.

Map coordinates match mapping.slam: x right, y forward, heading clockwise
from +y. Heights are relative to the scanner, not surveyed floor elevations.
Gravity is compared with the measured level reference; an unknown mount axis
is deliberately not guessed. Missing/invalid acceleration leaves a flat scan.

Wire format: ``points`` is base64, packed little-endian signed int16 XYZ.
Decode each coordinate as ``origin[axis] + integer * scale`` metres. At
30,000 points the geometry is 240 KB and visibility flags are 40 KB before
the small scene metadata, including JSON.
Only numpy and the standard library are needed on the Windows laptop.

``surface_flags`` is one base64-encoded byte per point: 1 = conservative
tilt/range risk, 2 = floor band/below, 4 = ceiling band/above. Zero marks a
point kept in the default view. The complete bounded cloud remains available
for the Floor/ceiling toggle. These are estimates, not semantic ground truth:
the fixed mount tilt axis and the sensor's changing height are not measured.
"""

import base64
import math
import threading
import time
from collections import OrderedDict

import numpy as np


LEVEL_REFERENCE = np.array([1.45, 1.33, 21.26], dtype=np.float64)
PERSON_TTL = 180.0


def _horizontal_levels(points):
    """Find separated, spatially broad height peaks, never cut by percentiles.

    The dominant near-zero layer of a 2-D scan is not a floor. Require a
    distinct peak above its shoulders and several square metres of XY
    support, so a flat scan, a vertical wall or a small tabletop cannot
    supply a floor/ceiling estimate by itself.
    """
    if len(points) < 400 or np.ptp(points[:, 2]) < .35:
        return None, None
    edges = np.arange(-4.0, 4.051, .05)
    histogram, _ = np.histogram(points[:, 2], edges)
    density = np.convolve(histogram, [1, 1, 1], mode="same")
    candidates = []
    for i in range(1, len(density) - 1):
        center = (edges[i] + edges[i + 1]) / 2
        if not (-2.0 <= center <= -.18 or .35 <= center <= 3.5):
            continue
        if density[i] < max(100, len(points) * .012):
            continue
        if density[i] < density[i - 1] or density[i] < density[i + 1]:
            continue
        shoulders = np.concatenate((histogram[max(0, i - 7):max(0, i - 3)],
                                    histogram[min(len(histogram), i + 4):i + 8]))
        if not len(shoulders) or density[i] < 2.2 * max(1.0, float(shoulders.mean()) * 3):
            continue
        band = points[np.abs(points[:, 2] - center) <= .075]
        if len(band) < 100:
            continue
        lo, hi = np.quantile(band[:, :2], [.05, .95], axis=0)
        span = hi - lo
        interior = band[np.all((band[:, :2] >= lo) & (band[:, :2] <= hi), axis=1), :2]
        cells = len(np.unique(np.floor(interior / .5).astype(np.int64), axis=0))
        area_cells = max(1.0, float(np.prod(np.ceil(span / .5))))
        if np.min(span) < 1.5 or cells < 20 or cells / area_cells < .35:
            continue
        core = interior[np.all((interior >= lo + span * .25) & (interior <= hi - span * .25), axis=1)]
        if len(np.unique(np.floor(core / .5).astype(np.int64), axis=0)) < 3:
            continue  # a perimeter of wall returns is not a horizontal surface
        candidates.append(float(np.median(band[:, 2])))
    floors = [v for v in candidates if v <= -.18]
    ceilings = [v for v in candidates if v >= .35]
    floor, ceiling = min(floors, default=None), max(ceilings, default=None)
    if floor is not None and ceiling is not None and ceiling - floor < 2.0:
        return None, None  # incompatible with the approximate room-height prior
    return floor, ceiling


def _classify_surfaces(points, required_heights, sensor_height_m, margin_m):
    floor, ceiling = _horizontal_levels(points)
    height = sensor_height_m
    if height is not None and floor is not None:
        height = min(height, max(.2, -floor))
    flags = np.zeros(len(points), dtype=np.uint8)
    if height is not None:
        flags[required_heights > height] |= 1
    if floor is not None:
        flags[points[:, 2] <= floor + margin_m] |= 2
    if ceiling is not None:
        flags[points[:, 2] >= ceiling - margin_m] |= 4
    return flags, floor, ceiling, height


def _number(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _tilt(acc):
    """Return the small rotation taking reference gravity to measured gravity.

    Using its transpose on sensor points levels them in the initial frame.
    Reject free-fall, large dynamic accelerations and implausible handheld
    tilt, rather than painting a spurious wall from one bad IMU sample.
    """
    if acc is None:
        return None
    try:
        gravity = np.asarray(acc, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None
    if gravity.shape != (3,) or not np.isfinite(gravity).all():
        return None
    length = float(np.linalg.norm(gravity))
    if not 12.0 <= length <= 31.0:  # about 0.56 to 1.45 g
        return None
    initial = LEVEL_REFERENCE / np.linalg.norm(LEVEL_REFERENCE)
    gravity = gravity / length
    cosine = float(np.clip(np.dot(initial, gravity), -1.0, 1.0))
    if cosine < math.cos(math.radians(30)):
        return None
    axis = np.cross(initial, gravity)
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return np.eye(3) + skew + skew @ skew / (1.0 + cosine)


class Cloud:
    def __init__(self, voxel_m=0.05, max_points=30000, *, sensor_height_m=1.0,
                 mount_tilt_deg=6.0, plane_margin_m=.15):
        if not math.isfinite(voxel_m) or voxel_m <= 0:
            raise ValueError("voxel_m must be positive and finite")
        if not 1 <= max_points <= 30000:
            raise ValueError("max_points must be between 1 and 30000")
        if sensor_height_m is not None and (not math.isfinite(sensor_height_m) or sensor_height_m <= 0):
            raise ValueError("sensor_height_m must be positive and finite, or None")
        if not math.isfinite(mount_tilt_deg) or not 0 <= mount_tilt_deg < 90:
            raise ValueError("mount_tilt_deg must be in [0, 90)")
        if not math.isfinite(plane_margin_m) or not 0 < plane_margin_m <= .5:
            raise ValueError("plane_margin_m must be in (0, .5]")
        self.voxel_m = float(voxel_m)
        self.max_points = int(max_points)
        self.sensor_height_m = sensor_height_m
        self.mount_tilt_deg = float(mount_tilt_deg)
        self.plane_margin_m = float(plane_margin_m)
        self._lock = threading.Lock()
        self._voxels = OrderedDict()
        self._version = 0
        self._packed = None
        self._pose = [0.0, 0.0, 0.0]
        self._last_scan = None
        self._accepted = self._rejected = 0
        self._scan_ok = True
        self._tilt_available = False

    def clear(self):
        """Start a new cloud when the operator resets the SLAM map."""
        with self._lock:
            self._voxels.clear()
            self._version += 1  # never reuse an old client's geometry version
            self._packed = None
            self._pose = [0.0, 0.0, 0.0]
            self._last_scan = None
            self._accepted = self._rejected = 0
            self._scan_ok = True
            self._tilt_available = False

    def add_scan(self, pose, ang_deg, dist_mm, acc=None, ok=True):
        """Add one accepted revolution, ignoring self-hits and invalid ranges."""
        with self._lock:
            self._last_scan = time.monotonic()
            self._scan_ok = bool(ok)
            if not ok:
                self._rejected += 1
                return
        try:
            pose = np.asarray(pose, dtype=np.float64)
            angles = np.asarray(ang_deg, dtype=np.float64).reshape(-1)
            distances = np.asarray(dist_mm, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError, OverflowError):
            return
        if pose.shape != (3,) or not np.isfinite(pose).all() or angles.shape != distances.shape:
            return
        valid = np.isfinite(angles) & np.isfinite(distances) & (distances > 250) & (distances <= 30000)
        if not valid.any():
            return
        angles = np.deg2rad(angles[valid])
        distances = distances[valid] / 1000.0
        points = np.column_stack((np.sin(angles) * distances, np.cos(angles) * distances,
                                  np.zeros(len(distances))))
        rotation = _tilt(acc)
        if acc is not None and rotation is None:
            # The IMU reads past 30 degrees or a shake (e.g. nodding the rig while the map is paused):
            # drawing this scan level would paint a false wall, so skip it.
            with self._lock:
                self._rejected += 1
            return
        required_height = np.zeros(len(points))
        if rotation is not None:
            points = points @ rotation  # row-vector form of rotation.T @ point
            gravity = np.asarray(acc, dtype=np.float64)
            cosine = float(gravity @ LEVEL_REFERENCE / (np.linalg.norm(gravity) * np.linalg.norm(LEVEL_REFERENCE)))
            tilt = math.acos(max(-1., min(1., cosine)))
            angle = min(math.pi / 2, tilt + math.radians(self.mount_tilt_deg + 1))
            # Same axis-independent bound as mapping.slam._trusted_range,
            # expressed as the sensor height a return would need to be safe.
            required_height = distances * math.sin(angle) / .95
        px, py, heading = pose
        c, s = math.cos(heading), math.sin(heading)
        x, y = points[:, 0].copy(), points[:, 1].copy()
        points[:, 0], points[:, 1] = px + c * x + s * y, py - s * x + c * y
        # An implausible external pose should not overflow a voxel key.
        if not np.isfinite(points).all() or np.max(np.abs(points)) > 1e6:
            return
        keys = np.floor(points / self.voxel_m).astype(np.int64)
        with self._lock:
            for key, point, height in zip(keys, points, required_height):
                key = tuple(int(v) for v in key)
                previous = self._voxels.get(key)
                # A close, trustworthy observation protects the same voxel
                # from being hidden just because a later ray is longer.
                if previous is not None:
                    height = min(float(height), previous[3])
                self._voxels[key] = (*map(float, point), float(height))
                self._voxels.move_to_end(key)
            while len(self._voxels) > self.max_points:
                self._voxels.popitem(last=False)
            self._pose = pose.tolist()
            self._accepted += 1
            self._tilt_available = rotation is not None
            self._version += 1
            self._packed = None

    def _pack(self):
        """Called under the lock; cache between polls until a new scan arrives."""
        if self._packed is not None:
            return self._packed
        if not self._voxels:
            self._packed = {"points": "", "count": 0, "origin": [0.0, 0.0, 0.0],
                            "scale": 0.01, "bounds": None, "surface_flags": "",
                            "visible_count": 0, "hidden_count": 0, "filtered_bounds": None,
                            "surface_filter": self._filter_info(None, None, self.sensor_height_m)}
            return self._packed
        values = np.asarray(list(self._voxels.values()), dtype=np.float64)
        points = values[:, :3]
        flags, floor, ceiling, height = _classify_surfaces(points, values[:, 3], self.sensor_height_m, self.plane_margin_m)
        visible = points[flags == 0]
        filtered_bounds = [np.round(visible.min(axis=0), 3).tolist(), np.round(visible.max(axis=0), 3).tolist()] if len(visible) else None
        lo, hi = points.min(axis=0), points.max(axis=0)
        origin = np.round((lo + hi) / 2.0, 3)
        span = float(np.max(np.abs(points - origin)))
        scale = max(0.01, math.ceil(span / 32760.0 * 1000.0) / 1000.0)
        packed = np.rint((points - origin) / scale).astype("<i2")
        self._packed = {"points": base64.b64encode(packed.tobytes()).decode("ascii"),
                        "count": len(points), "origin": origin.tolist(), "scale": scale,
                        "bounds": [np.round(lo, 3).tolist(), np.round(hi, 3).tolist()],
                        "surface_flags": base64.b64encode(flags.tobytes()).decode("ascii"),
                        "visible_count": len(visible), "hidden_count": int(np.count_nonzero(flags)),
                        "filtered_bounds": filtered_bounds,
                        "surface_filter": self._filter_info(floor, ceiling, height)}
        return self._packed

    def _filter_info(self, floor, ceiling, height):
        return {"floor_z": None if floor is None else round(floor, 3),
                "ceiling_z": None if ceiling is None else round(ceiling, 3),
                "margin_m": self.plane_margin_m, "sensor_height_m": height,
                "mount_tilt_deg": self.mount_tilt_deg, "method": "conservative"}

    def snapshot(self, persons=None, pose=None, path=None, nodes=None):
        """Return a bounded JSON-ready scene; never retain caller-owned metadata."""
        now = time.time()
        people = []
        source = persons.values() if isinstance(persons, dict) else (persons if persons is not None else [])
        for person in source:
            if not isinstance(person, dict):
                continue
            age = _number(person.get("age"))
            seen = _number(person.get("seen"))
            if age is None and seen is not None:
                age = max(0.0, now - seen)
            if age is not None and age > PERSON_TTL:
                continue
            p = {"id": str(person.get("id", "?"))[:32], "x": _number(person.get("x")),
                 "y": _number(person.get("y")), "ps": person.get("ps") if person.get("ps") in ("U", "D") else None,
                 "src": str(person.get("src") or "")[:20], "r": _number(person.get("r")),
                 "age": round(max(0.0, age), 1) if age is not None else None}
            kp = person.get("kp")
            if isinstance(kp, (list, tuple)) and len(kp) == 34:
                p["kp"] = [int(value) if (value := _number(v)) is not None and (value == -1 or 0 <= value <= 99) else -1
                           for v in kp]
            people.append(p)
            if len(people) >= 128:
                break
        route = []
        if path is not None:
            # Keep both endpoints even for a long walk, with a bounded payload.
            step = max(1, math.ceil(max(0, len(path) - 1) / 799))
            candidates = list(path[::step])
            if len(path) > 1 and (len(path) - 1) % step:
                candidates.append(path[-1])
            for point in candidates:
                if len(point) < 2:
                    continue
                x, y = _number(point[0]), _number(point[1])
                if x is not None and y is not None:
                    route.append([round(x, 3), round(y, 3)])
        relays = {}
        for name, node in list((nodes or {}).items())[:32]:
            if not isinstance(node, dict):
                continue
            x, y = _number(node.get("x")), _number(node.get("y"))
            if x is not None and y is not None:
                relays[str(name)[:32]] = {"x": round(x, 3), "y": round(y, 3)}
        with self._lock:
            current_pose = [_number(v) for v in pose] if pose is not None and len(pose) == 3 else None
            if current_pose is None or any(v is None for v in current_pose):
                current_pose = list(self._pose)
            return {"v": self._version, "encoding": "i16le", "voxel": self.voxel_m,
                    **self._pack(), "pose": current_pose, "persons": people, "path": route, "nodes": relays,
                    "accepted": self._accepted, "rejected": self._rejected, "scan_ok": self._scan_ok,
                    "scan_age": round(time.monotonic() - self._last_scan, 1) if self._last_scan is not None else None,
                    "tilt": self._tilt_available}
