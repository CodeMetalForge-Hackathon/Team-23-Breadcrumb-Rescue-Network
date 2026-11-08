import math
import time

import numpy as np
import pytest

from mapping.floorplan import extract

RES = 0.05
ORIGIN = (-15.0, -15.0)
N = 600


def _cell(x, y):
    return int((y - ORIGIN[1]) / RES), int((x - ORIGIN[0]) / RES)


def _two_rooms(angle=0.0, noise=True, seed=1):
    """Two rooms side by side (8 x 5 m and 5 x 5 m) sharing a wall with a 1 m door, rotated by angle.
    Returns (grid, true walls in map metres)."""
    rng = np.random.default_rng(seed)
    img = np.full((N, N), 127, np.uint8)
    walls = [((-6, -2.5), (7, -2.5)), ((-6, 2.5), (7, 2.5)), ((-6, -2.5), (-6, 2.5)), ((7, -2.5), (7, 2.5)),
             ((2, -2.5), (2, -0.5)), ((2, 0.5), (2, 2.5))]
    ca, sa = math.cos(angle), math.sin(angle)

    def rot(p):
        return ca * p[0] - sa * p[1], sa * p[0] + ca * p[1]

    # free interior
    for x in np.arange(-5.95, 6.96, RES / 2):
        for y in np.arange(-2.45, 2.46, RES / 2):
            iy, ix = _cell(*rot((x, y)))
            img[iy, ix] = 0
    truth = []
    for a, b in walls:
        a, b = rot(a), rot(b)
        truth.append((a, b))
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (RES / 3)) + 1
        for t in np.linspace(0, 1, n):
            x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            if noise and rng.random() < 0.1:                      # LiDAR misses some cells
                continue
            jx, jy = (rng.normal(0, 0.02, 2) if noise else (0, 0))
            iy, ix = _cell(x + jx, y + jy)
            img[iy, ix] = 255
    if noise:
        # floor-hit spray: isolated dots out in unknown space, and a few speckles inside the rooms
        for _ in range(300):
            iy, ix = rng.integers(0, N, 2)
            img[iy, ix] = 255
        for _ in range(40):
            iy, ix = _cell(*rot((rng.uniform(-5.5, 6.5), rng.uniform(-2, 2))))
            img[iy, ix] = 255
    return img, truth


def _covered(truth_seg, walls, tol=0.12):
    """Fraction of a true wall that lies within tol of some extracted, roughly parallel wall."""
    (ax, ay), (bx, by) = truth_seg
    L = math.hypot(bx - ax, by - ay)
    tx, ty = (bx - ax) / L, (by - ay) / L
    pts = [(ax + tx * s, ay + ty * s) for s in np.arange(0.1, L - 0.1, 0.05)]
    hit = 0
    for px, py in pts:
        for x1, y1, x2, y2 in walls:
            wl = math.hypot(x2 - x1, y2 - y1)
            if wl == 0:
                continue
            ux, uy = (x2 - x1) / wl, (y2 - y1) / wl
            if abs(ux * tx + uy * ty) < math.cos(math.radians(10)):
                continue
            s = (px - x1) * ux + (py - y1) * uy
            d = abs((px - x1) * -uy + (py - y1) * ux)
            if -0.1 <= s <= wl + 0.1 and d <= tol:
                hit += 1
                break
    return hit / len(pts)


@pytest.mark.parametrize("angle_deg", [0, 20, -35])
def test_two_rooms_recovered(angle_deg):
    img, truth = _two_rooms(math.radians(angle_deg))
    out = extract(img, ORIGIN, RES)
    want = math.radians(angle_deg)
    err = (out["angle"] - want + math.pi / 4) % (math.pi / 2) - math.pi / 4
    assert abs(err) < math.radians(2), out["angle"]
    for seg in truth:
        assert _covered(seg, out["walls"]) > 0.85, seg
    # no noise walls: everything extracted lies on a true wall
    total = sum(math.hypot(w[2] - w[0], w[3] - w[1]) for w in out["walls"])
    true_total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in truth)
    assert total < 1.15 * true_total
    assert len(out["walls"]) <= 12


def test_door_gap_kept():
    img, _ = _two_rooms(0.0, noise=False)
    walls = extract(img, ORIGIN, RES)["walls"]
    inner = [w for w in walls if abs(w[0] - 2) < 0.15 and abs(w[2] - 2) < 0.15]
    assert len(inner) == 2                       # the shared wall is split by the 1 m door
    for w in inner:
        lo, hi = sorted((w[1], w[3]))
        assert hi <= -0.35 or lo >= 0.35


def test_empty_and_noise_only():
    img = np.full((200, 200), 127, np.uint8)
    assert extract(img, ORIGIN, RES)["walls"] == []
    rng = np.random.default_rng(0)
    img[rng.integers(0, 200, 200), rng.integers(0, 200, 200)] = 255
    img[100:110, 100:110] = 0
    assert extract(img, ORIGIN, RES)["walls"] == []


def test_output_is_json_friendly():
    import json
    img, _ = _two_rooms(0.3)
    out = extract(img, ORIGIN, RES)
    json.dumps(out)
    assert all(len(w) == 4 for w in out["walls"])


def test_fast_enough():
    img, _ = _two_rooms(0.2)
    extract(img, ORIGIN, RES)
    t = time.perf_counter()
    extract(img, ORIGIN, RES)
    assert time.perf_counter() - t < 0.5        # target ~0.1 s; loose bound for slow CI machines
