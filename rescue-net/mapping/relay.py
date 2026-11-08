"""Conservative relay suggestions from a discrete occupancy map.

The radio model is a planning heuristic, not a measured link budget. Coordinates
are metres; grid row zero is the minimum map y coordinate.
"""
from dataclasses import dataclass
import math

import numpy as np


FREE = 0
UNKNOWN = 127
OCCUPIED = 255


@dataclass(frozen=True)
class RelayModel:
    """Uncalibrated planning constants; tune together after radio measurements."""

    reference_margin_db: float = 30.0  # assumed margin at one metre
    distance_loss_db: float = 20.0     # loss per decade of distance
    wall_loss_db: float = 8.0         # loss per contiguous wall segment
    trigger_margin_db: float = 8.0
    min_spacing_m: float = 1.0
    reached_radius_m: float = 0.6
    path_step_m: float = 0.25
    placement_clearance_m: float = 0.20


MODEL = RelayModel()


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


def _quality(grid, origin, res, anchor, point):
    distance = math.dist(anchor, point)
    walls, unknown = _ray_stats(grid, origin, res, anchor, point)
    margin = (MODEL.reference_margin_db
              - MODEL.distance_loss_db * math.log10(max(1.0, distance))
              - MODEL.wall_loss_db * walls)
    return margin, distance, walls, unknown


def _placeable(grid, origin, res, point):
    ix, iy = _cell(point, origin, res)
    height, width = grid.shape
    if not (0 <= ix < width and 0 <= iy < height) or grid[iy, ix] != FREE:
        return False, False
    radius = math.ceil(MODEL.placement_clearance_m / res)
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
    for index, row in enumerate(path):
        try:
            if len(row) not in (2, 4) or not all(math.isfinite(float(v)) for v in row):
                raise ValueError
            xy = row if len(row) == 2 else row[1:3]
            points.append((float(xy[0]), float(xy[1])))
        except (TypeError, ValueError, OverflowError):
            raise ValueError(f"path[{index}] must be finite (x, y) or (t, x, y, th)") from None
    return points


def _sample_path(points, res):
    if not points:
        return
    yield points[0]
    step = min(MODEL.path_step_m, res)
    for start, end in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist(start, end) / step))
        for i in range(1, n + 1):
            fraction = i / n
            yield (start[0] + fraction * (end[0] - start[0]),
                   start[1] + fraction * (end[1] - start[1]))


def suggest_relays(grid, origin_xy, res, nodes, path):
    """Return ``[{x, y, reason}, ...]`` for ``(x, y)`` or ``(t, x, y, th)`` samples.

    Start from the known node nearest the first path point, then use the last
    node reached along the walk. A suggested relay becomes the next assumed
    anchor. Place at the last known-free point with enough predicted margin,
    before the next sample crosses the threshold. Suggestions assume earlier
    suggestions are installed; empty output is not a coverage guarantee.
    """
    grid, origin, res = _map_args(grid, origin_xy, res)
    if not hasattr(nodes, "items"):
        raise ValueError("nodes must be a mapping of names to (x, y)")
    if not nodes:
        return []
    known = [_xy(point, f"node {name!r}") for name, point in nodes.items()]
    points = _path_xy(path)
    if not points:
        return []
    anchor = min(known, key=lambda point: math.dist(point, points[0]))
    suggestions, candidate = [], None
    for point in _sample_path(points, res):
        reached = [node for node in known if math.dist(node, point) <= MODEL.reached_radius_m]
        if reached:
            nearest = min(reached, key=lambda node: math.dist(node, point))
            if nearest != anchor:
                anchor, candidate = nearest, None
        margin, distance, walls, unknown = _quality(grid, origin, res, anchor, point)
        if margin < MODEL.trigger_margin_db and candidate is not None:
            location, safe_margin, nearby_unknown = candidate
            caution = "; crosses unmapped space" if unknown else ""
            if nearby_unknown:
                caution += "; placement borders unmapped space"
            reason = (f"Heuristic, unmeasured: next path point predicts {margin:.1f} dB "
                      f"margin at {distance:.1f} m through {walls} wall segment(s); "
                      f"place before the {MODEL.trigger_margin_db:g} dB threshold "
                      f"(here {safe_margin:.1f} dB){caution}.")
            suggestions.append({"x": location[0], "y": location[1], "reason": reason})
            known.append(location)
            anchor, candidate = location, None
            margin, distance, walls, unknown = _quality(grid, origin, res, anchor, point)
        placeable, nearby_unknown = _placeable(grid, origin, res, point)
        if (placeable and margin >= MODEL.trigger_margin_db
                and distance >= MODEL.min_spacing_m
                and all(math.dist(node, point) >= MODEL.min_spacing_m for node in known)):
            candidate = (point, margin, nearby_unknown)
    return suggestions
