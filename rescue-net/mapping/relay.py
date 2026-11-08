"""Conservative relay planning with explicit radio calibration and geometry.

Coordinates/distances are metres, RSSI and thresholds are dBm, and wall loss is
dB per crossing. Grid row zero is the minimum map y. Uncalibrated defaults are a
planning heuristic, not a measured link budget. No model file is read implicitly.
"""
from dataclasses import dataclass
import json
import math

import numpy as np


FREE = 0
UNKNOWN = 127
OCCUPIED = 255


@dataclass(frozen=True)
class RelayModel:
    """RSSI = p0_dbm - 10*n*log10(max(distance_m, 1)) - wall_loss_db*walls.

    ``calibrated`` describes the supplied model's provenance; setting it does
    not validate a fit. Defaults reproduce the original relay trigger exactly.
    Placement parameters remain in metres and can be tuned independently.
    """

    p0_dbm: float = -50.0
    path_loss_exponent: float = 2.0
    wall_loss_db: float = 8.0
    threshold_dbm: float = -72.0
    calibrated: bool = False
    min_spacing_m: float = 1.0
    reached_radius_m: float = 0.6
    path_step_m: float = 0.25
    placement_clearance_m: float = 0.20

    def __post_init__(self):
        for name in ("p0_dbm", "path_loss_exponent", "wall_loss_db", "threshold_dbm",
                     "min_spacing_m", "reached_radius_m", "path_step_m",
                     "placement_clearance_m"):
            original = getattr(self, name)
            try:
                value = float(original)
            except (TypeError, ValueError, OverflowError):
                raise ValueError(f"{name} must be finite") from None
            if isinstance(original, (bool, np.bool_)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if name in ("path_loss_exponent", "min_spacing_m", "path_step_m") and value <= 0:
                raise ValueError(f"{name} must be positive")
            if name in ("wall_loss_db", "reached_radius_m", "placement_clearance_m") and value < 0:
                raise ValueError(f"{name} must be nonnegative")
            object.__setattr__(self, name, value)
        if not isinstance(self.calibrated, (bool, np.bool_)):
            raise ValueError("calibrated must be a boolean")
        object.__setattr__(self, "calibrated", bool(self.calibrated))

    @classmethod
    def from_dict(cls, data):
        """Load explicit model fields, ignoring fitter diagnostics/metadata."""
        required = {"p0_dbm", "path_loss_exponent", "wall_loss_db", "threshold_dbm", "calibrated"}
        if not isinstance(data, dict) or not required <= data.keys():
            raise ValueError("model must provide p0_dbm, path_loss_exponent, wall_loss_db, "
                             "threshold_dbm and calibrated")
        return cls(**{name: data[name] for name in cls.__dataclass_fields__ if name in data})

    # Read-only aliases retain the previous MODEL's public margin constants.
    @property
    def reference_margin_db(self):
        return self.p0_dbm + 80.

    @property
    def distance_loss_db(self):
        return 10. * self.path_loss_exponent

    @property
    def trigger_margin_db(self):
        return self.threshold_dbm + 80.


MODEL = RelayModel()


def load_model(path):
    """Read a UTF-8 JSON model explicitly; malformed/missing input raises."""
    with open(path, encoding="utf-8") as handle:
        return RelayModel.from_dict(json.load(handle))


def _map_args(grid, origin_xy, res):
    grid = np.asarray(grid)
    if grid.ndim != 2 or not grid.size or grid.dtype != np.uint8:
        raise ValueError("grid must be a nonempty 2-D uint8 array")
    if not np.isin(grid, (FREE, UNKNOWN, OCCUPIED)).all():
        raise ValueError("grid values must be 0 (free), 127 (unknown), or 255 (occupied)")
    origin = _xy(origin_xy, "origin_xy")
    try:
        resolution = float(res)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("res must be a finite positive number of metres per cell") from None
    if isinstance(res, bool) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("res must be a finite positive number of metres per cell")
    return grid, origin, resolution


def _xy(value, name):
    try:
        if len(value) != 2:
            raise ValueError
        result = tuple(float(v) for v in value)
        if not all(math.isfinite(v) for v in result):
            raise ValueError
        return result
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"{name} must contain two finite coordinates") from None


def _cell(point, origin, res):
    return (math.floor((point[0] - origin[0]) / res),
            math.floor((point[1] - origin[1]) / res))


def _ray_cells(start, end, origin, res):
    """Traverse cells in ray order; an exact corner crossing advances both axes."""
    x, y = _cell(start, origin, res)
    end_x, end_y = _cell(end, origin, res)
    dx, dy = end[0] - start[0], end[1] - start[1]
    sx = 1 if dx > 0 else -1 if dx < 0 else 0
    sy = 1 if dy > 0 else -1 if dy < 0 else 0
    tx = ((origin[0] + (x + (sx > 0)) * res - start[0]) / dx
          if sx else math.inf)
    ty = ((origin[1] + (y + (sy > 0)) * res - start[1]) / dy
          if sy else math.inf)
    dtx = res / abs(dx) if sx else math.inf
    dty = res / abs(dy) if sy else math.inf
    # The bound also prevents floating-point corner ties from looping forever.
    for _ in range(abs(end_x - x) + abs(end_y - y) + 1):
        yield x, y
        if (x, y) == (end_x, end_y):
            return
        if math.isclose(tx, ty, rel_tol=1e-12, abs_tol=1e-12):
            x, y, tx, ty = x + sx, y + sy, tx + dtx, ty + dty
        elif tx < ty:
            x, tx = x + sx, tx + dtx
        else:
            y, ty = y + sy, ty + dty


def _ray_stats(grid, origin, res, start, end):
    walls, unknown, in_wall = 0, False, False
    height, width = grid.shape
    for ix, iy in _ray_cells(start, end, origin, res):
        value = grid[iy, ix] if 0 <= ix < width and 0 <= iy < height else UNKNOWN
        occupied = value == OCCUPIED
        walls += int(occupied and not in_wall)
        unknown |= value == UNKNOWN
        in_wall = occupied
    return walls, bool(unknown)


def count_wall_segments(grid, origin_xy, res, start_xy, end_xy):
    """Count contiguous occupied runs, not pixels or estimated physical walls."""
    grid, origin, res = _map_args(grid, origin_xy, res)
    return _ray_stats(grid, origin, res, _xy(start_xy, "start_xy"),
                      _xy(end_xy, "end_xy"))[0]


def _prepare_walls(walls):
    """Merge overlapping near-parallel representations once per planning call.

    The complete group's perpendicular spread must stay within 0.15 m, with
    directions within 2 degrees. This prevents a chain of close segments from
    collapsing walls separated by more than the tolerance. Separated supports
    are not joined across gaps. Geometry alone cannot distinguish two physical
    walls within this explicit duplicate tolerance.
    """
    try:
        rows = np.asarray(walls, dtype=float)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("walls must be finite [x1, y1, x2, y2] segments") from None
    if rows.shape == (0,):
        return np.empty((0, 4), dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 4 or not np.isfinite(rows).all():
        raise ValueError("walls must be finite [x1, y1, x2, y2] segments")
    rows = rows.copy()
    reverse = (rows[:, 0] > rows[:, 2]) | ((rows[:, 0] == rows[:, 2]) & (rows[:, 1] > rows[:, 3]))
    rows[reverse] = rows[reverse][:, [2, 3, 0, 1]]
    lengths = np.linalg.norm(rows[:, 2:] - rows[:, :2], axis=1)
    if np.any(lengths <= 1e-9):
        raise ValueError("walls must have nonzero length")
    order = np.lexsort((rows[:, 3], rows[:, 2], rows[:, 1], rows[:, 0], -lengths))
    count = len(rows)
    bases, axes = np.empty((count, 2)), np.empty((count, 2))
    lows, highs, offsets_low, offsets_high = (np.empty(count) for _ in range(4))
    used = 0
    for index in order:
        a, b = rows[index, :2], rows[index, 2:]
        axis = (b - a) / lengths[index]
        delta_a, delta_b = a - bases[:used], b - bases[:used]
        pa, pb = np.sum(delta_a * axes[:used], axis=1), np.sum(delta_b * axes[:used], axis=1)
        oa = axes[:used, 0] * delta_a[:, 1] - axes[:used, 1] * delta_a[:, 0]
        ob = axes[:used, 0] * delta_b[:, 1] - axes[:used, 1] * delta_b[:, 0]
        lo, hi = np.minimum(pa, pb), np.maximum(pa, pb)
        offset_lo = np.minimum(offsets_low[:used], np.minimum(oa, ob))
        offset_hi = np.maximum(offsets_high[:used], np.maximum(oa, ob))
        # Shallow-angle walls that actually cross are distinct geometry, even
        # when their endpoint separation fits the duplicate tolerance.
        crossing_projection = np.divide(pa * ob - pb * oa, ob - oa,
                                        out=np.zeros(used), where=np.abs(ob - oa) > 1e-12)
        proper_intersection = ((oa * ob < -1e-16)
                               & (crossing_projection > lows[:used] + 1e-9)
                               & (crossing_projection < highs[:used] - 1e-9))
        compatible = ((np.abs(axes[:used] @ axis) >= math.cos(math.radians(2.)))
                      & (hi >= lows[:used] - 1e-9) & (lo <= highs[:used] + 1e-9)
                      & (offset_hi - offset_lo <= .15 + 1e-9) & ~proper_intersection)
        matches = np.flatnonzero(compatible)
        if len(matches):
            j = int(matches[0])
            lows[j], highs[j] = min(lows[j], lo[j]), max(highs[j], hi[j])
            offsets_low[j], offsets_high[j] = offset_lo[j], offset_hi[j]
        else:
            bases[used], axes[used] = a, axis
            lows[used], highs[used] = 0., lengths[index]
            offsets_low[used] = offsets_high[used] = 0.
            used += 1
    normal = np.column_stack((-axes[:used, 1], axes[:used, 0]))
    centres = bases[:used] + normal * ((offsets_low[:used] + offsets_high[:used]) / 2.)[:, None]
    return np.column_stack((centres + axes[:used] * lows[:used, None],
                            centres + axes[:used] * highs[:used, None]))


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _geometric_crossings(start, end, walls):
    if not len(walls):
        return 0
    start, end = np.asarray(start), np.asarray(end)
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length <= 1e-9:
        return 0
    a, b = walls[:, :2], walls[:, 2:]
    segments = b - a
    denominator = _cross(direction, segments)
    usable = np.abs(denominator) > 1e-10 * length * np.linalg.norm(segments, axis=1)
    t = np.divide(_cross(a - start, segments), denominator,
                  out=np.zeros(len(walls)), where=usable)
    u = np.divide(_cross(a - start, direction), denominator,
                  out=np.zeros(len(walls)), where=usable)
    on_link = usable & (t > 1e-9) & (t < 1. - 1e-9)
    interior = np.flatnonzero(on_link & (u > 1e-9) & (u < 1. - 1e-9))
    # Residual overlapping fragments can survive a bridging merge. Coalesce
    # exact collinear events only: applying the 0.15 m tolerance again here
    # would join groups whose original combined spread exceeds that bound.
    accepted = []
    for index in interior:
        segment = segments[index]
        if any(abs(t[index] - t[j]) * length <= 1e-8
               and abs(float(_cross(segment, segments[j]))) <=
               1e-10 * np.linalg.norm(segment) * np.linalg.norm(segments[j])
               for j in accepted):
            continue
        accepted.append(int(index))
    count = len(accepted)
    endpoints = np.flatnonzero(on_link & ((np.abs(u) <= 1e-9) | (np.abs(u - 1.) <= 1e-9)))
    # Touching an isolated wall tip or a convex corner is tangency. A shared
    # vertex crosses a boundary only when its branches extend to both sides.
    endpoint_groups = []
    for index in endpoints:
        for group in endpoint_groups:
            if abs(t[index] - t[group[0]]) * length <= 1e-8:
                group.append(int(index))
                break
        else:
            endpoint_groups.append([int(index)])
    for group in endpoint_groups:
        sides = []
        for index in group:
            remote = b[index] if abs(u[index]) <= 1e-9 else a[index]
            sides.append(float(_cross(direction, remote - start)))
        if min(sides) < -1e-9 and max(sides) > 1e-9:
            # A longer copy of a straight fragmented wall may already count it.
            represented = any(abs(t[group[0]] - t[j]) * length <= 1e-8
                              and all(abs(float(_cross(segments[i], segments[j]))) <=
                                      1e-10 * np.linalg.norm(segments[i])
                                      * np.linalg.norm(segments[j]) for i in group)
                              for j in accepted)
            count += not represented
    return int(count)


def count_wall_crossings(start_xy, end_xy, walls):
    """Count geometric wall crossings after 0.15 m parallel duplicate merging.

    Link endpoints and collinear travel are excluded. A wall-tip touch is not
    penetration; a shared vertex counts once if branches extend to opposite
    sides of the link, zero for same-side tangency. Proper interior crossings
    of nonparallel walls count separately, even at the same position. Parallel
    walls more than 0.15 m apart remain distinct. ``walls=[]`` returns zero.
    """
    return _geometric_crossings(_xy(start_xy, "start_xy"), _xy(end_xy, "end_xy"),
                                _prepare_walls(walls))


def _quality(grid, origin, res, anchor, point, walls, model):
    distance = math.dist(anchor, point)
    occupied_runs, unknown = _ray_stats(grid, origin, res, anchor, point)
    crossings = occupied_runs if walls is None else _geometric_crossings(anchor, point, walls)
    rssi = (model.p0_dbm - 10. * model.path_loss_exponent * math.log10(max(1., distance))
            - model.wall_loss_db * crossings)
    return rssi, distance, crossings, unknown


def _placeable(grid, origin, res, point, model):
    ix, iy = _cell(point, origin, res)
    height, width = grid.shape
    if not (0 <= ix < width and 0 <= iy < height) or grid[iy, ix] != FREE:
        return False, False
    radius = math.ceil(model.placement_clearance_m / res)
    nearby_unknown = False
    for y in range(iy - radius, iy + radius + 1):
        for x in range(ix - radius, ix + radius + 1):
            if not (0 <= x < width and 0 <= y < height):
                nearby_unknown = True
                continue
            if grid[y, x] == OCCUPIED:
                return False, False
            nearby_unknown |= grid[y, x] == UNKNOWN
    return True, bool(nearby_unknown)


def _path_xy(path):
    points = []
    try:
        iterator = iter(path)
    except TypeError:
        raise ValueError("path must contain (x, y) or (t, x, y, th) rows") from None
    for index, row in enumerate(iterator):
        try:
            if len(row) not in (2, 4) or not all(math.isfinite(float(v)) for v in row):
                raise ValueError
            xy = row if len(row) == 2 else row[1:3]
            points.append((float(xy[0]), float(xy[1])))
        except (TypeError, ValueError, OverflowError):
            raise ValueError(f"path[{index}] must be finite (x, y) or (t, x, y, th)") from None
    return points


def _sample_path(points, res, model):
    if not points:
        return
    yield points[0]
    step = min(model.path_step_m, res)
    for start, end in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist(start, end) / step))
        for i in range(1, n + 1):
            fraction = i / n
            yield (start[0] + fraction * (end[0] - start[0]),
                   start[1] + fraction * (end[1] - start[1]))


def _measurements(measured):
    observations = {}
    if measured is None:
        return observations
    try:
        iterator = iter(measured)
    except TypeError:
        raise ValueError("measured must contain (x, y, rssi_dbm, node_name) rows") from None
    for index, row in enumerate(iterator):
        try:
            if len(row) != 4 or not isinstance(row[3], str) or not row[3]:
                raise ValueError
            values = tuple(float(v) for v in row[:3])
            if not all(math.isfinite(v) for v in values):
                raise ValueError
        except (TypeError, ValueError, OverflowError):
            raise ValueError(f"measured[{index}] must be finite (x, y, rssi_dbm, node_name)") from None
        observations.setdefault(row[3], []).append(values)
    return {name: np.asarray(values) for name, values in observations.items()}


def _link_quality(grid, origin, res, anchor, point, walls, observations, model):
    name, location, real = anchor
    predicted, distance, crossings, unknown = _quality(grid, origin, res, location, point, walls, model)
    rssi, source = predicted, "model RSSI"
    nearby = observations.get(name) if real else None
    if nearby is not None:
        distances = np.sum((nearby[:, :2] - point) ** 2, axis=1)
        # Reverse argmin gives the last input observation priority on exact ties.
        best = len(distances) - 1 - int(np.argmin(distances[::-1]))
        if distances[best] <= .5 ** 2 + 1e-12:
            rssi = float(nearby[best, 2])
            source = f"measured RSSI from real node {name!r}"
    return rssi, distance, crossings, unknown, predicted, source


def suggest_relays(grid, origin_xy, res, nodes, path, walls=None, measured=None, *, model=None):
    """Return ``[{x, y, reason}, ...]`` for ``(x, y)`` or ``(t, x, y, th)`` samples.

    Start from the known node nearest the first path point, then use the last
    node reached along the walk. A suggested relay becomes the next assumed
    anchor. Place at the last known-free point with sufficient RSSI,
    before the next sample crosses the threshold. Suggestions assume earlier
    suggestions are installed; empty output is not a coverage guarantee.

    ``walls`` is a sequence of [x1,y1,x2,y2] metre segments, prepared once per
    call; None uses occupied-grid runs, while [] explicitly means no geometric
    walls. Grid occupancy always governs safe placement and unknown caution.
    ``measured`` rows are (x,y,rssi_dbm,node_name). Only the current REAL named
    anchor can use observations within 0.5 m; closest wins, last input wins
    ties. These spatial observations have no timestamps or expiry: callers
    must supply relevant current readings. They are never reused for a
    hypothetical relay, including on a retraced path. The optional RelayModel
    must be supplied explicitly; the default is uncalibrated.
    """
    grid, origin, res = _map_args(grid, origin_xy, res)
    if not hasattr(nodes, "items"):
        raise ValueError("nodes must be a mapping of names to (x, y)")
    model = MODEL if model is None else model
    if not isinstance(model, RelayModel):
        raise ValueError("model must be a RelayModel")
    geometry = None if walls is None else _prepare_walls(walls)
    observations = _measurements(measured)
    known = [(name, _xy(point, f"node {name!r}"), True) for name, point in nodes.items()]
    points = _path_xy(path)
    if not points or not known:
        return []
    anchor = min(known, key=lambda node: math.dist(node[1], points[0]))
    suggestions, candidate = [], None
    for point in _sample_path(points, res, model):
        reached = [node for node in known if math.dist(node[1], point) <= model.reached_radius_m]
        if reached:
            nearest = min(reached, key=lambda node: math.dist(node[1], point))
            if nearest != anchor:
                anchor, candidate = nearest, None
        rssi, distance, crossings, unknown, predicted, source = _link_quality(
            grid, origin, res, anchor, point, geometry, observations, model)
        if rssi < model.threshold_dbm and candidate is not None:
            location, safe_rssi, nearby_unknown, safe_source, safe_unknown = candidate
            caution = "; crosses unmapped space" if unknown or safe_unknown else ""
            if nearby_unknown:
                caution += "; placement borders unmapped space"
            status = "Calibrated model" if model.calibrated else "Heuristic, unmeasured (uncalibrated model)"
            if not model.calibrated and (source != "model RSSI" or safe_source != "model RSSI"):
                status = "Model uncalibrated; local measurement override"
            reason = (f"{status}: next path point {source} is {rssi:.1f} dBm "
                      f"at {distance:.1f} m through {crossings} wall segment(s) "
                      f"(model {predicted:.1f} dBm); place at the last good free breadcrumb "
                      f"before the {model.threshold_dbm:g} dBm threshold "
                      f"(here {safe_rssi:.1f} dBm, {safe_source}){caution}.")
            suggestions.append({"x": location[0], "y": location[1], "reason": reason})
            anchor = (None, location, False)
            known.append(anchor)
            candidate = None
            rssi, distance, crossings, unknown, predicted, source = _link_quality(
                grid, origin, res, anchor, point, geometry, observations, model)
        placeable, nearby_unknown = _placeable(grid, origin, res, point, model)
        if (placeable and rssi >= model.threshold_dbm
                and distance >= model.min_spacing_m
                and all(math.dist(node[1], point) >= model.min_spacing_m for node in known)):
            candidate = (point, rssi, nearby_unknown, source, unknown)
    return suggestions
