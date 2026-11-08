"""Room-style floor plan: wall line segments from a Slam2D occupancy grid (numpy + stdlib only).

    extract(img, origin, res) -> {"walls": [[x1, y1, x2, y2], ...], "angle": rad, "cells": n_kept}

``img`` is exactly ``Slam2D.grid()``: uint8, 0 free, 127 unknown, 255 occupied; img[iy, ix] covers
(ox + (ix + .5) * res, oy + (iy + .5) * res), row 0 = minimum y. Walls are in map metres.

Steps
1. Denoise: keep occupied cells that touch free space (within ``FREE_NEAR_M``), have an occupied
   neighbour, and belong to a connected group at least ``MIN_BLOB_M`` across. LiDAR floor/ceiling
   sprays and single stray returns die here.
2. Dominant wall direction: structure tensor of the occupied mask, angles folded mod 90 deg
   (indoor rooms are mostly Manhattan), then refined by a small search for the sharpest projection. ``angle`` is in (-pi/4, pi/4], measured like atan2(y, x).
3. In that rotated frame, cells are split into the two wall families by local direction. Each family
   is scanned for dense lines (a band ``BAND_M`` thick), each line is cut into runs where the gap
   is over ``GAP_M``, and runs shorter than ``MIN_WALL_M`` are dropped. Parallel neighbours within
   ``MERGE_M`` are merged (double-sided walls).
4. Leftover cells (diagonal walls) get a small RANSAC pass.
"""
import math

import numpy as np

OCC, FREE = 255, 0
FREE_NEAR_M = 0.15      # an occupied cell must have free space this close (it was seen from somewhere)
MIN_BLOB_M = 0.30       # connected groups smaller than this across are noise
BAND_M = 0.10           # half-thickness of a wall line band
GAP_M = 0.30            # bigger gaps split a wall into two segments
MIN_WALL_M = 0.40       # shortest wall segment kept
MIN_FILL = 0.35         # cells per res along a segment (rejects sparse sprays lining up by chance)
MERGE_M = 0.15          # parallel segments closer than this (and overlapping) become one
FAMILY_TOL = math.radians(30)
RANSAC_ITERS = 80
DIAG_MIN_WALL_M = 0.60   # diagonal walls need more evidence than Manhattan ones
DIAG_FILL = 0.6
BLOB_DENSITY = 12         # occupied cells in a 5x5 box above which a cell is clutter, not a wall


def _shift(a, dy, dx):
    """a shifted by (dy, dx) with zero fill."""
    out = np.zeros_like(a)
    h, w = a.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = a[ys, xs]
    return out


def _box(a, r, dtype=np.float32):
    """Sum over a (2r+1)^2 box, via an integral image."""
    p = np.pad(np.asarray(a, dtype), ((r + 1, r), (r + 1, r)))
    s = p.cumsum(0, dtype=dtype).cumsum(1, dtype=dtype)
    k = 2 * r + 1
    return s[k:, k:] - s[:-k, k:] - s[k:, :-k] + s[:-k, :-k]


def _dilate(mask, r):
    return _box(mask, r, np.int32) > 0


def _components(mask):
    """8-connected components of a sparse boolean mask -> list of (iy array, ix array)."""
    ys, xs = np.nonzero(mask)
    if len(ys) == 0:
        return []
    h, w = mask.shape
    lin = ys * w + xs
    index = {int(v): i for i, v in enumerate(lin)}
    parent = list(range(len(lin)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    # union with already-indexed neighbours (half of the 8-neighbourhood is enough)
    for dy, dx in ((0, 1), (1, -1), (1, 0), (1, 1)):
        ny, nx = ys + dy, xs + dx
        ok = (ny < h) & (nx >= 0) & (nx < w)
        cand = (ny * w + nx)[ok]
        src = np.nonzero(ok)[0]
        hit = mask[ny[ok], nx[ok]]
        for i, c in zip(src[hit], cand[hit]):
            a, b = find(int(i)), find(index[int(c)])
            if a != b:
                parent[a] = b
    roots = np.array([find(i) for i in range(len(lin))])
    order = np.argsort(roots, kind="stable")
    r = roots[order]
    cuts = np.nonzero(np.diff(r))[0] + 1
    return [(ys[g], xs[g]) for g in np.split(order, cuts)]


def denoise(img, res):
    """Boolean mask of occupied cells that look like real structure."""
    occ = img == OCC
    if not occ.any():
        return occ
    free_near = _dilate(img == FREE, max(1, int(round(FREE_NEAR_M / res))))
    nb = _box(occ, 1, np.int32) - occ                       # occupied 8-neighbours
    keep = occ & free_near & (nb >= 1)
    out = np.zeros_like(keep)
    min_cells = MIN_BLOB_M / res
    for ys, xs in _components(keep):
        if max(ys.max() - ys.min(), xs.max() - xs.min()) + 1 >= min_cells:
            out[ys, xs] = True
    return out


def _orientation(mask):
    """Per-cell wall direction (rad, mod pi) and coherence 0..1 from the structure tensor."""
    f = _box(mask, 1) / 9.0
    gy, gx = np.gradient(f)
    jxx, jyy, jxy = _box(gx * gx, 2), _box(gy * gy, 2), _box(gx * gy, 2)
    normal = 0.5 * np.arctan2(2 * jxy, jxx - jyy)          # gradient direction = wall normal
    tr = jxx + jyy
    coh = np.sqrt((jxx - jyy) ** 2 + 4 * jxy ** 2) / np.maximum(tr, 1e-12)
    return normal + math.pi / 2, coh, tr


def dominant_angle(mask, orientation=None):
    """Dominant wall direction mod 90 deg, in (-pi/4, pi/4]. 0 if nothing to go on."""
    if mask.sum() < 3:
        return 0.0
    direction, coh, tr = orientation if orientation is not None else _orientation(mask)
    d, w = direction[mask], (coh * tr)[mask]
    z = np.sum(w * np.exp(4j * d))
    if abs(z) < 1e-12:
        return 0.0
    return float(np.angle(z) / 4)


def refine_angle(x, y, angle, res, span=math.radians(4), step=math.radians(0.25)):
    """Sharpen the coarse angle: the one whose projections pile cells into the fewest, densest lines
    (sum of squared histogram counts along both axes). Removes the raster bias of the structure tensor."""
    best, best_a = -1.0, angle
    for a in np.arange(angle - span, angle + span + step / 2, step):
        ca, sa = math.cos(a), math.sin(a)
        score = 0.0
        for p in (-sa * x + ca * y, ca * x + sa * y):
            h = np.bincount(np.floor((p - p.min()) / res).astype(np.int64))
            score += float(np.dot(h, h))
        if score > best:
            best, best_a = score, a
    return float((best_a + math.pi / 4) % (math.pi / 2) - math.pi / 4)


def _runs(u, gap):
    """Split sorted u into runs separated by more than gap -> list of index slices."""
    if len(u) == 0:
        return []
    cuts = np.nonzero(np.diff(u) > gap)[0] + 1
    bounds = np.concatenate(([0], cuts, [len(u)]))
    return [slice(a, b) for a, b in zip(bounds[:-1], bounds[1:])]


def _family_lines(u, v, res):
    """Wall segments (u0, u1, v) for cells whose walls run along u. Returns (segments, used mask)."""
    segs = []
    used = np.zeros(len(u), bool)
    if len(u) == 0:
        return segs, used
    band = max(BAND_M, res)
    min_cells = MIN_WALL_M / res * MIN_FILL
    v0 = v.min()
    nb = int((v.max() - v0) / res) + 3
    bins = np.floor((v - v0) / res).astype(int) + 1
    alive = np.ones(len(u), bool)
    for _ in range(400):
        hist = np.bincount(bins[alive], minlength=nb).astype(float)
        sm = hist + np.concatenate(([0], hist[:-1])) + np.concatenate((hist[1:], [0]))
        b = int(np.argmax(sm))
        if sm[b] < min_cells:
            break
        centre = v0 + (b - 0.5) * res
        sel = alive & (np.abs(v - centre) <= band)
        idx = np.nonzero(sel)[0]
        alive[idx] = False
        order = idx[np.argsort(u[idx])]
        for sl in _runs(u[order], GAP_M):
            run = order[sl]
            length = u[run[-1]] - u[run[0]] + res
            if length >= MIN_WALL_M and len(run) >= length / res * MIN_FILL:
                segs.append([float(u[run[0]] - res / 2), float(u[run[-1]] + res / 2), float(np.median(v[run])),
                             len(run)])
                used[run] = True
    return _merge(segs), used


def _merge(segs):
    """Merge parallel segments that are within MERGE_M of each other and overlap or nearly touch."""
    segs = sorted(segs, key=lambda s: (s[2], s[0]))
    changed = True
    while changed:
        changed = False
        out = []
        for s in segs:
            for t in out:
                if abs(t[2] - s[2]) <= MERGE_M and s[0] <= t[1] + GAP_M and t[0] <= s[1] + GAP_M:
                    n = t[3] + s[3]
                    t[2] = (t[2] * t[3] + s[2] * s[3]) / n
                    t[0], t[1], t[3] = min(t[0], s[0]), max(t[1], s[1]), n
                    changed = True
                    break
            else:
                out.append(list(s))
        segs = out
    return segs


def _ransac(px, py, pdir, res, rng):
    """Straight segments among leftover (diagonal) cells. A cell only counts as an inlier when its own
    local wall direction agrees with the line, so sprays of rays do not fan out into fake walls."""
    segs = []
    pts = np.stack([px, py], 1)
    pdir = np.asarray(pdir, float)
    min_len = DIAG_MIN_WALL_M
    min_cells = int(min_len / res * MIN_FILL) + 2
    for _ in range(12):
        n = len(pts)
        if n < min_cells:
            break
        i, j = rng.integers(0, n, RANSAC_ITERS), rng.integers(0, n, RANSAC_ITERS)
        d = pts[j] - pts[i]
        ln = np.hypot(d[:, 0], d[:, 1])
        ok = ln >= 3 * res
        if not ok.any():
            break
        i, d, ln = i[ok], d[ok], ln[ok]
        t = d / ln[:, None]
        nrm = np.stack([-t[:, 1], t[:, 0]], 1)
        dist = np.abs((pts[:, 0][None] - pts[i, 0][:, None]) * nrm[:, 0:1]
                      + (pts[:, 1][None] - pts[i, 1][:, None]) * nrm[:, 1:2])
        ang = np.arctan2(t[:, 1], t[:, 0])
        da = np.abs(np.mod(pdir[None] - ang[:, None] + math.pi / 2, math.pi) - math.pi / 2)
        inl = (dist <= BAND_M) & (da <= math.radians(20))
        k = int(np.argmax(inl.sum(1)))
        best = inl[k]
        if best.sum() < min_cells:
            break
        q = pts[best]
        c = q.mean(0)
        _, _, vt = np.linalg.svd(q - c, full_matrices=False)       # refit on inliers
        tt = vt[0]
        s = (q - c) @ tt
        order = np.argsort(s)
        for sl in _runs(s[order], GAP_M):
            run = order[sl]
            a, b = s[run[0]], s[run[-1]]
            if b - a + res >= min_len and len(run) >= (b - a + res) / res * DIAG_FILL:
                p1, p2 = c + a * tt, c + b * tt
                segs.append([float(p1[0]), float(p1[1]), float(p2[0]), float(p2[1])])
        pts, pdir = pts[~best], pdir[~best]
    return segs


def extract(img, origin, res, diagonal=True, seed=0):
    img = np.asarray(img)
    ox, oy = float(origin[0]), float(origin[1])
    res = float(res)
    known = np.nonzero(img != 127)
    if len(known[0]) == 0:
        return {"walls": [], "angle": 0.0, "cells": 0}
    pad = 4                                                   # work on the known bounding box only
    y0, x0 = max(int(known[0].min()) - pad, 0), max(int(known[1].min()) - pad, 0)
    y1, x1 = int(known[0].max()) + pad + 1, int(known[1].max()) + pad + 1
    sub = img[y0:y1, x0:x1]
    mask = denoise(sub, res)
    n = int(mask.sum())
    if n == 0:
        return {"walls": [], "angle": 0.0, "cells": 0}
    orient = _orientation(mask)
    direction, coh, _ = orient
    angle = dominant_angle(mask, orient)
    iy, ix = np.nonzero(mask)
    x = ox + (ix + x0 + 0.5) * res
    y = oy + (iy + y0 + 0.5) * res
    angle = refine_angle(x, y, angle, res)
    ca, sa = math.cos(angle), math.sin(angle)
    u, v = ca * x + sa * y, -sa * x + ca * y                  # rotated frame: family A runs along u
    rel = np.mod(direction[iy, ix] - angle, math.pi)          # local wall direction relative to angle
    d_a = np.minimum(rel, math.pi - rel)                      # 0 = along u
    d_b = np.abs(rel - math.pi / 2)                           # 0 = along v
    weak = coh[iy, ix] < 0.3                                  # corners / blobs: let both families try
    in_a = weak | (d_a <= FAMILY_TOL)
    in_b = weak | (d_b <= FAMILY_TOL)
    walls = []
    used = np.zeros(n, bool)
    ia = np.nonzero(in_a)[0]
    segs, ua = _family_lines(u[ia], v[ia], res)
    used[ia[ua]] = True
    for a, b, vv, _ in segs:
        walls.append((a, vv, b, vv))
    ib = np.nonzero(in_b)[0]
    segs, ub = _family_lines(v[ib], u[ib], res)
    used[ib[ub]] = True
    for a, b, uu, _ in segs:
        walls.append((uu, a, uu, b))
    out = []
    for p, q, r, s in walls:                                  # rotate back to the map frame
        out.append([ca * p - sa * q, sa * p + ca * q, ca * r - sa * s, sa * r + ca * s])
    if diagonal:
        dense = _box(mask, 2, np.int32)[iy, ix] > BLOB_DENSITY      # clutter blobs, not thin walls
        left = ~used & ~weak & ~dense & (d_a > FAMILY_TOL) & (d_b > FAMILY_TOL)
        if left.sum() > 0:
            out += _ransac(x[left], y[left], direction[iy, ix][left], res, np.random.default_rng(seed))
    walls = [[round(float(c), 3) for c in w] for w in out]
    return {"walls": walls, "angle": round(angle, 4), "cells": n}
