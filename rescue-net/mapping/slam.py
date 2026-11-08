"""Local 2-D scan-to-map SLAM using NumPy and the standard library only.

Coordinates are deliberately the same as the LiDAR/people-bearing feed: zero
bearing looks along +y, +90 degrees looks along +x, and positive headings turn
clockwise. ``pose`` is (x metres, y metres, heading radians). A sensor point is
``(sin(bearing)*range, cos(bearing)*range)``. The first accepted scan defines the
world frame; there is no GPS, compass, odometry or global loop closure here.

The matcher searches an accumulated endpoint likelihood map, then integrates
only a sufficiently good match. Free-space rays and occupied endpoints are
stored separately as bounded evidence. The matching landmarks persist: this
is intended for mostly static indoor structure, not a changing crowd or doors.
Symmetric rooms, long featureless corridors, severe scan distortion and motion
beyond the bounded search can still make localization ambiguous. ``ok`` is a
quality gate, not a mathematical guarantee of the correct pose.

``grid()`` returns ``(uint8_array, (xmin, ymin), resolution)``. The array is
indexed [iy, ix], row zero is the minimum world y, and values are exactly 0
(free), 127 (unknown), and 255 (occupied). It is a snapshot, safe for a caller
to modify. ``to_map(bearing_deg, range_m)`` projects a *sensor observation into
world metres*; it does not return pixel indices.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np


_LEVEL = np.array((1.45, 1.33, 21.26), dtype=np.float64)
_LEVEL_NORM = float(np.linalg.norm(_LEVEL))
_LEVEL /= _LEVEL_NORM
_COS_TILT = math.cos(math.radians(10.0))


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Slam2D:
    """Bounded local correlative LiDAR mapper.

    Parameters use metres except scan input distances, which use millimetres.
    ``update(angles, distances, t=None, acc=None)`` accepts equal-length 1-D
    arrays (360 binned returns or roughly 500 raw points both work). Invalid,
    zero, short and out-of-range returns are discarded. Malformed/insufficient
    scans are rejected, without altering pose, path or map. Optional ``t`` is
    a finite timestamp in seconds, increasing between accepted timestamped
    scans; it enables prediction and a wider search after dropped scans.

    ``acc`` is an optional three-vector in accelerometer *counts*, approximately
    21.33 counts/g. An invalid magnitude or a direction more than ten degrees
    from (1.45, 1.33, 21.26) rejects the whole scan. Missing acceleration is
    allowed. No gravity/tilt compensation is guessed from bad measurements.

    Result keys: x, y, th, ok, score, reason. Score is a normalized [0, 1]
    endpoint-map agreement (the initial seed has score 1). Rejected results
    report the unchanged accepted pose. ``path`` contains accepted scans only,
    as (timestamp, x, y, th). Without ``t`` the path uses wall-clock timestamps
    and the motion search assumes a nominal 10 Hz scan stream.

    The map is fixed-size and centred on the seed. It does not resize or reset
    automatically when tracking is lost. Motion search is limited to 2 m and
    60 degrees around a bounded prediction; long outages may need a new mapper.
    """

    def __init__(self, res=.05, size_m=40, min_range=.25, max_range=12):
        values = np.asarray((res, size_m, min_range, max_range), dtype=float)
        res, size_m, min_range, max_range = map(float, values)
        if (not np.isfinite(values).all() or res <= 0 or size_m <= 0
                or min_range < 0 or max_range <= min_range):
            raise ValueError("finite positive map dimensions and ordered ranges are required")
        n = int(math.ceil(float(size_m) / float(res)))
        if not 16 <= n <= 4096:
            raise ValueError("map side must contain between 16 and 4096 cells")
        self.res = float(res)
        self.size_m = n * self.res
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.origin_xy = (-self.size_m / 2., -self.size_m / 2.)
        self._n = n
        self._evidence = np.zeros((n, n), dtype=np.int16)
        self._field = np.zeros((n, n), dtype=np.uint8)
        self._pose = np.zeros(3, dtype=np.float64)
        self._path = []
        self._velocity = np.zeros(3, dtype=np.float64)
        self._last_explicit_time = None
        self._rejected = 0
        self._lock = threading.RLock()

        # One broad likelihood field serves all search levels. Squaring its
        # values in fine matching makes the final score more selective.
        self._sigma = max(.16, 2.0 * self.res)
        radius = int(math.ceil(3.0 * self._sigma / self.res))
        d = np.arange(-radius, radius + 1, dtype=np.int32)
        dx, dy = np.meshgrid(d, d)
        dist2 = (dx * dx + dy * dy) * self.res ** 2
        weight = np.rint(255.0 * np.exp(-dist2 / (2.0 * self._sigma ** 2)))
        use = weight >= 4
        self._kernel_x = dx[use]
        self._kernel_y = dy[use]
        self._kernel_v = weight[use].astype(np.uint8)

    @property
    def pose(self):
        with self._lock:
            return tuple(float(v) for v in self._pose)

    @property
    def path(self):
        with self._lock:
            return list(self._path)

    @property
    def rejected(self):
        """Number of update calls rejected since construction."""
        with self._lock:
            return self._rejected

    def grid(self):
        """Return a uint8 map snapshot, lower-left world origin, and metres/cell."""
        with self._lock:
            out = np.full(self._evidence.shape, 127, dtype=np.uint8)
            out[self._evidence < 0] = 0
            out[self._evidence >= 2] = 255
            return out, self.origin_xy, self.res

    def to_map(self, bearing_deg, range_m):
        """Project scalar or broadcastable array observations to world (x, y)."""
        bearing, distance = np.broadcast_arrays(np.asarray(bearing_deg, dtype=float),
                                               np.asarray(range_m, dtype=float))
        if (not np.isfinite(bearing).all() or not np.isfinite(distance).all()
                or np.any(distance < 0)):
            raise ValueError("bearing and nonnegative range must be finite")
        with self._lock:
            angle = np.deg2rad(bearing) + self._pose[2]
            x = self._pose[0] + np.sin(angle) * distance
            y = self._pose[1] + np.cos(angle) * distance
        if x.ndim == 0:
            return float(x), float(y)
        return x, y

    def _result(self, ok, score=0., reason=""):
        if not ok:
            self._rejected += 1
        score = float(score) if math.isfinite(float(score)) else 0.
        return dict(x=float(self._pose[0]), y=float(self._pose[1]),
                    th=float(self._pose[2]), ok=bool(ok),
                    score=max(0., min(1., score)), reason=reason)

    @staticmethod
    def _tilt_reason(acc):
        if acc is None:
            return None
        try:
            a = np.asarray(acc, dtype=np.float64)
        except (TypeError, ValueError, OverflowError):
            return "invalid_acc"
        if a.shape != (3,) or not np.isfinite(a).all():
            return "invalid_acc"
        norm = float(np.linalg.norm(a))
        if not .7 * _LEVEL_NORM <= norm <= 1.3 * _LEVEL_NORM:
            return "unreliable_acc"
        if float(a @ _LEVEL) / norm < _COS_TILT:
            return "tilted"
        return None

    def _points(self, ang_deg, dist_mm):
        try:
            angle = np.asarray(ang_deg, dtype=np.float64)
            distance = np.asarray(dist_mm, dtype=np.float64)
        except (ValueError, TypeError, OverflowError):
            return None, "malformed_scan"
        if (angle.ndim != 1 or distance.ndim != 1 or angle.shape != distance.shape
                or angle.size > 20000):
            return None, "malformed_scan"
        valid = (np.isfinite(angle) & np.isfinite(distance)
                 & (distance >= self.min_range * 1000.)
                 & (distance <= self.max_range * 1000.) & (distance > 0))
        angle = angle[valid] % 360.
        distance = distance[valid] * .001
        if angle.size < 40 or np.unique(np.floor(angle)).size < 40:
            return None, "insufficient_returns"
        order = np.argsort(angle)
        if order.size > 360:
            order = order[np.linspace(0, order.size - 1, 360, dtype=int)]
        a = np.deg2rad(angle[order])
        d = distance[order]
        points = np.column_stack((np.sin(a) * d, np.cos(a) * d)).astype(np.float32)
        centred = points - points.mean(axis=0)
        eigenvalues = np.linalg.eigvalsh(centred.T @ centred / len(points))
        if eigenvalues[0] < .015 or eigenvalues[1] < .15:
            return None, "degenerate_scan"
        return points, None

    def update(self, ang_deg, dist_mm, t=None, acc=None):
        """Match one complete scan and integrate it only on acceptance."""
        with self._lock:
            reason = self._tilt_reason(acc)
            if reason:
                return self._result(False, reason=reason)
            try:
                stamp = time.time() if t is None else float(t)
            except (TypeError, ValueError, OverflowError):
                return self._result(False, reason="invalid_time")
            if not math.isfinite(stamp):
                return self._result(False, reason="invalid_time")
            if t is not None and self._last_explicit_time is not None:
                dt = stamp - self._last_explicit_time
                if dt <= 0:
                    return self._result(False, reason="non_monotonic_time")
            else:
                dt = .1
            points, reason = self._points(ang_deg, dist_mm)
            if reason:
                return self._result(False, reason=reason)

            if not self._path:
                cells = self._world_cells(points)
                inside = ((cells[:, 0] >= 0) & (cells[:, 0] < self._n)
                          & (cells[:, 1] >= 0) & (cells[:, 1] < self._n))
                if np.count_nonzero(inside) < 40:
                    return self._result(False, reason="out_of_map")
                candidate, score = self._pose.copy(), 1.
            else:
                candidate, score, inliers = self._match(points, dt)
                if score < .60 or np.mean(inliers) < .60:
                    return self._result(False, score, "poor_match")
                # Agreement must span several bearing sectors, not one small
                # object that happens to fit a previously observed wall.
                sectors = np.floor((np.arctan2(points[:, 0], points[:, 1])
                                    % (2 * np.pi)) * (12 / (2 * np.pi))).astype(int)
                if np.unique(sectors[inliers]).size < 4:
                    return self._result(False, score, "limited_overlap")
                delta = candidate - self._pose
                delta[2] = _wrap(delta[2])
                max_step = 2.0 * min(dt, 2.) + .20
                max_turn = math.radians(120.) * min(dt, 2.) + math.radians(10.)
                if np.linalg.norm(delta[:2]) > max_step or abs(delta[2]) > max_turn:
                    return self._result(False, score, "implausible_motion")
                centre = self._world_cells(candidate[None, :2])[0]
                if np.any(centre < 0) or np.any(centre >= self._n):
                    return self._result(False, score, "out_of_map")

            # All rejection paths precede every mutation of accepted state.
            old_pose = self._pose.copy()
            self._integrate(points, candidate)
            self._pose[:] = candidate
            self._pose[2] = _wrap(self._pose[2])
            if self._path and .02 <= dt <= 2.:
                delta = self._pose - old_pose
                delta[2] = _wrap(delta[2])
                velocity = delta / dt
                speed = np.linalg.norm(velocity[:2])
                if speed > 1.5:
                    velocity[:2] *= 1.5 / speed
                velocity[2] = np.clip(velocity[2], -math.radians(120), math.radians(120))
                self._velocity[:] = .65 * velocity + .35 * self._velocity
            else:
                self._velocity[:] = 0.
            self._last_explicit_time = stamp if t is not None else None
            self._path.append((stamp, *self.pose))
            return self._result(True, score, "seed" if len(self._path) == 1 else "matched")

    def _world_cells(self, xy):
        return np.floor((xy - np.asarray(self.origin_xy)) / self.res).astype(np.int32)

    def _likelihoods(self, points, candidates, interpolate=False):
        """Endpoint field samples; chunks bound temporary allocation in search."""
        ct = np.cos(candidates[:, 2]).astype(np.float32)[:, None]
        st = np.sin(candidates[:, 2]).astype(np.float32)[:, None]
        gx = (ct * points[:, 0] + st * points[:, 1]
              + candidates[:, 0, None] - self.origin_xy[0]) / self.res - .5
        gy = (-st * points[:, 0] + ct * points[:, 1]
              + candidates[:, 1, None] - self.origin_xy[1]) / self.res - .5
        if not interpolate:
            ix = np.rint(gx).astype(np.int32)
            iy = np.rint(gy).astype(np.int32)
            valid = (ix >= 0) & (iy >= 0) & (ix < self._n) & (iy < self._n)
            values = self._field[np.clip(iy, 0, self._n - 1),
                                 np.clip(ix, 0, self._n - 1)].astype(np.float32)
        else:
            ix = np.floor(gx).astype(np.int32)
            iy = np.floor(gy).astype(np.int32)
            valid = (ix >= 0) & (iy >= 0) & (ix < self._n - 1) & (iy < self._n - 1)
            fx, fy = gx - ix, gy - iy
            ix = np.clip(ix, 0, self._n - 2)
            iy = np.clip(iy, 0, self._n - 2)
            values = ((1. - fx) * (1. - fy) * self._field[iy, ix]
                      + fx * (1. - fy) * self._field[iy, ix + 1]
                      + (1. - fx) * fy * self._field[iy + 1, ix]
                      + fx * fy * self._field[iy + 1, ix + 1])
        return values * valid * (1. / 255.)

    def _scores(self, points, candidates, prediction, power=1, interpolate=False):
        scores = np.empty(len(candidates), dtype=np.float64)
        for start in range(0, len(candidates), 768):
            part = candidates[start:start + 768]
            values = self._likelihoods(points, part, interpolate)
            if power == 2:
                values *= values
            scores[start:start + len(part)] = values.mean(axis=1)
        # Small motion prior resolves flat/quantized ties; it is not part of
        # the public match score or an excuse to accept a low-overlap scan.
        d = candidates - prediction
        d[:, 2] = (d[:, 2] + np.pi) % (2 * np.pi) - np.pi
        scores -= .004 * np.sum(d[:, :2] ** 2, axis=1) + .02 * d[:, 2] ** 2
        return scores

    @staticmethod
    def _candidates(centres, dx, dy, da):
        x, y, a = np.meshgrid(dx, dy, da, indexing="ij")
        offsets = np.column_stack((x.ravel(), y.ravel(), a.ravel()))
        return (np.asarray(centres)[:, None, :] + offsets[None, :, :]).reshape(-1, 3)

    def _match(self, points, dt):
        horizon = min(dt, 1.)
        prediction = self._pose + self._velocity * horizon
        radius = min(2., max(.60, 1.25 * dt + .25))
        yaw_radius = min(math.radians(60), max(math.radians(18), math.radians(70) * dt))
        nx = int(math.ceil(radius / .20))
        na = int(math.ceil(yaw_radius / math.radians(3.)))
        coarse = self._candidates([prediction], np.arange(-nx, nx + 1) * .20,
                                  np.arange(-nx, nx + 1) * .20,
                                  np.arange(-na, na + 1) * math.radians(3.))
        small = points[np.linspace(0, len(points) - 1, min(100, len(points)), dtype=int)]
        scores = self._scores(small, coarse, prediction)
        # Keep separated hypotheses so a coarse cell near a doorway does not
        # displace the correct peak before refinement.
        top = np.argsort(scores)[::-1][:80]
        peaks = []
        for i in top:
            q = coarse[i]
            if all(np.linalg.norm(q[:2] - p[:2]) >= .18
                   or abs(q[2] - p[2]) >= math.radians(2.9) for p in peaks):
                peaks.append(q)
                if len(peaks) == 3:
                    break
        fine = self._candidates(peaks, np.arange(-2, 3) * .05,
                                np.arange(-2, 3) * .05,
                                np.arange(-4, 5) * math.radians(.75))
        medium = points[np.linspace(0, len(points) - 1, min(200, len(points)), dtype=int)]
        score = self._scores(medium, fine, prediction, power=2)
        best = fine[int(np.argmax(score))]
        finest = self._candidates([best], np.arange(-2, 3) * .025,
                                  np.arange(-2, 3) * .025,
                                  np.arange(-3, 4) * math.radians(.25))
        score = self._scores(medium, finest, prediction, power=2, interpolate=True)
        best = finest[int(np.argmax(score))].copy()
        best[2] = _wrap(best[2])
        values = self._likelihoods(points, best[None, :], interpolate=True)[0]
        return best, float(np.mean(values ** 2)), values >= .60

    def _integrate(self, points, pose):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = np.column_stack((c * points[:, 0] + s * points[:, 1] + pose[0],
                                 -s * points[:, 0] + c * points[:, 1] + pose[1]))
        endpoints = self._world_cells(world)
        inside = ((endpoints[:, 0] >= 0) & (endpoints[:, 0] < self._n)
                  & (endpoints[:, 1] >= 0) & (endpoints[:, 1] < self._n))
        hits = np.unique(endpoints[inside, 1] * self._n + endpoints[inside, 0])

        # DDA sampling at no more than one dominant-axis cell per step. Each
        # crossed cell gets at most one free-space vote per complete scan.
        delta = world - pose[:2]
        steps = np.maximum(1, np.ceil(np.max(np.abs(delta), axis=1) / self.res).astype(int))
        j = np.arange(int(steps.max()), dtype=np.float32)[None, :]
        f = j / steps[:, None]
        ix = np.floor((pose[0] + delta[:, 0, None] * f - self.origin_xy[0]) / self.res).astype(int)
        iy = np.floor((pose[1] + delta[:, 1, None] * f - self.origin_xy[1]) / self.res).astype(int)
        valid = ((j < steps[:, None]) & (ix >= 0) & (iy >= 0)
                 & (ix < self._n) & (iy < self._n))
        free = np.unique(iy[valid] * self._n + ix[valid])
        evidence = self._evidence.ravel()
        evidence[free] = np.maximum(evidence[free] - 1, -20)
        evidence[hits] = np.minimum(evidence[hits] + 5, 20)

        hits = hits[evidence[hits] >= 2]
        ix = hits[:, None] % self._n + self._kernel_x
        iy = hits[:, None] // self._n + self._kernel_y
        valid = (ix >= 0) & (iy >= 0) & (ix < self._n) & (iy < self._n)
        values = np.broadcast_to(self._kernel_v, ix.shape)
        np.maximum.at(self._field.ravel(), (iy * self._n + ix)[valid], values[valid])
