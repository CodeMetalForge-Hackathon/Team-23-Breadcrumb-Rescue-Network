"""Independent geometric acceptance tests for mapping.

The oracle casts rays against a known, asymmetric room.  It does not use the
mapper's point conversion, matcher, rasterizer, or motion estimate.  Run this
file directly for a controlled trajectory accuracy/timing report (timing is
intentionally not asserted by pytest).
"""

from __future__ import annotations

import copy
import json
import math
import time

import numpy as np
import pytest

from mapping.slam import Slam2D


LEVEL_ACC = np.array([1.45, 1.33, 21.26])

# Unequal walls, an alcove, and an offset column remove rotational symmetries.
ROOM = np.array([
    [-4.0, -3.0], [5.0, -3.0], [5.0, 2.5], [3.6, 2.5],
    [3.6, 7.0], [-4.0, 7.0],
])
COLUMN = np.array([[1.6, 3.0], [2.2, 3.0], [2.2, 3.6], [1.6, 3.6]])
STARTS = np.concatenate([ROOM, COLUMN])
ENDS = np.concatenate([np.roll(ROOM, -1, axis=0), np.roll(COLUMN, -1, axis=0)])


def raycast(pose=(0.0, 0.0, 0.0), *, raw=False, frame=0, noisy=False):
    """Return sensor bearings in degrees and mm ranges at a known world pose.

    Heading is clockwise from north.  Raw acquisition has repeated integer
    bearings, irregular angular samples, and a changing start angle.  These
    details prevent accidentally testing only an indexed 360-value scan.
    """
    rng = np.random.default_rng(2087 + frame)
    if raw:
        angles = (np.arange(517) * (360.0 / 517) + frame * 0.37) % 360.0
        angles = np.round(angles) % 360.0
        angles = np.roll(angles, frame * 13)
    else:
        angles = np.arange(360, dtype=float)
    radians = np.deg2rad(angles + math.degrees(pose[2]))
    rays = np.column_stack([np.sin(radians), np.cos(radians)])
    edges = ENDS - STARTS
    offset = STARTS - np.asarray(pose[:2])
    # Intersection p + range*ray == segment_start + fraction*edge.
    denominator = rays[:, 0, None] * edges[None, :, 1] - rays[:, 1, None] * edges[None, :, 0]
    numerator = offset[:, 0] * edges[:, 1] - offset[:, 1] * edges[:, 0]
    fraction_numerator = offset[None, :, 0] * rays[:, 1, None] - offset[None, :, 1] * rays[:, 0, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        ranges = numerator[None, :] / denominator
        fractions = fraction_numerator / denominator
    valid = (ranges > 0) & (fractions >= 0) & (fractions <= 1) & (np.abs(denominator) > 1e-12)
    distances = np.min(np.where(valid, ranges, np.inf), axis=1) * 1000.0
    if noisy:
        distances += rng.normal(0, 5.0, len(distances))
    return angles, np.round(distances)


def angle_error(actual, expected):
    return abs(math.atan2(math.sin(actual - expected), math.cos(actual - expected)))


def result_pose(result):
    assert {"x", "y", "th", "ok", "score"} <= result.keys()
    assert all(math.isfinite(float(result[k])) for k in ("x", "y", "th", "score"))
    assert 0 <= result["score"] <= 1
    return np.array([result["x"], result["y"], result["th"]], dtype=float)


def snapshot(slam):
    return copy.deepcopy(slam.pose), copy.deepcopy(slam.path), copy.deepcopy(slam.grid())


def assert_unchanged(slam, before):
    pose, path, grid = before
    np.testing.assert_equal(slam.pose, pose)
    np.testing.assert_equal(slam.path, path)
    actual_grid, actual_origin, actual_res = slam.grid()
    expected_grid, expected_origin, expected_res = grid
    np.testing.assert_array_equal(actual_grid, expected_grid)
    np.testing.assert_array_equal(actual_origin, expected_origin)
    assert actual_res == expected_res


def run_trajectory(*, raw=False, dropped=False, noisy=True, sample_period=0.1):
    """One metre/second walk: straight, clockwise arc, counterclockwise arc.

    Consecutive missing acquisitions cause a 0.3 s / 0.3 m observation gap.
    Each endpoint is computed from ground-truth motion, never mapper output.
    """
    slam = Slam2D()
    truth = np.zeros(3)
    rows = []
    for frame in range(round(6.5 / sample_period) + 1):
        elapsed = frame * sample_period
        if frame:
            yaw_rate = 0.0 if elapsed < 1.7 else (30.0 if elapsed < 3.7 else (-30.0 if elapsed < 5.7 else 0.0))
            yaw_step = yaw_rate * sample_period
            mid_heading = truth[2] + math.radians(yaw_step) / 2
            truth[:2] += sample_period * np.array([math.sin(mid_heading), math.cos(mid_heading)])
            truth[2] += math.radians(yaw_step)
        if dropped and frame in {10, 11, 28, 29, 46, 47}:
            continue
        angles, distances = raycast(truth, raw=raw, frame=frame, noisy=noisy)
        started = time.perf_counter_ns()
        result = slam.update(angles, distances, t=1000.0 + elapsed, acc=LEVEL_ACC)
        elapsed_ms = (time.perf_counter_ns() - started) / 1e6
        predicted = result_pose(result)
        rows.append({
            "frame": frame, "ok": bool(result["ok"]), "score": float(result["score"]),
            "true_pose": truth.copy().tolist(), "pose": predicted.tolist(),
            "translation_error_m": float(np.linalg.norm(predicted[:2] - truth[:2])),
            "heading_error_deg": math.degrees(angle_error(predicted[2], truth[2])),
            "latency_ms": elapsed_ms,
        })
    return slam, rows


def test_raycast_oracle_known_cardinal_distances():
    # This guards the test fixture against a swapped bearing convention.
    angles, distances = raycast()
    assert len(angles) == 360
    assert distances[[0, 90, 180, 270]].tolist() == [7000, 5000, 3000, 4000]


def test_bootstrap_is_initial_frame_and_path_is_timestamped():
    slam = Slam2D()
    result = slam.update(*raycast(), t=123.5, acc=LEVEL_ACC)
    assert result["ok"]
    np.testing.assert_allclose(result_pose(result), [0, 0, 0], atol=1e-8)
    assert len(slam.path) == 1
    np.testing.assert_allclose(slam.path[0], [123.5, 0, 0, 0], atol=1e-8)


def test_grid_encoding_and_world_origin_place_asymmetric_walls_correctly():
    slam = Slam2D(res=0.05, size_m=40)
    assert slam.update(*raycast(), t=1.0)["ok"]
    grid, origin, resolution = slam.grid()
    assert isinstance(grid, np.ndarray)
    assert grid.dtype == np.uint8
    assert grid.shape == (800, 800)
    assert resolution == pytest.approx(0.05)
    np.testing.assert_allclose(origin, [-20, -20])
    assert set(np.unique(grid)) <= {0, 127, 255}

    def neighborhood(x, y, radius=1):
        col, row = np.floor((np.array([x, y]) - origin) / resolution).astype(int)
        return grid[row - radius:row + radius + 1, col - radius:col + radius + 1]

    for wall_point in [(0, 7), (5, 0), (0, -3), (-4, 0)]:
        assert 255 in neighborhood(*wall_point), wall_point
    for free_point in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        assert 0 in neighborhood(*free_point), free_point
    assert np.all(neighborhood(10, 10) == 127)


def test_to_map_returns_world_metres_with_current_pose():
    slam = Slam2D()
    assert slam.update(*raycast(), t=0.0)["ok"]
    for bearing, expected in [(0, (0, 2)), (90, (2, 0)), (180, (0, -2)), (270, (-2, 0))]:
        np.testing.assert_allclose(slam.to_map(bearing, 2), expected, atol=1e-8)
    for frame in range(1, 9):
        true_pose = (frame * 0.05, frame * 0.05, math.radians(frame * 5))
        result = slam.update(*raycast(true_pose, frame=frame), t=frame * 0.1)
        assert result["ok"], (frame, result)
    for bearing in [0, 90, 180, 270]:
        world_heading = true_pose[2] + math.radians(bearing)
        expected = np.array(true_pose[:2]) + 2 * np.array([math.sin(world_heading), math.cos(world_heading)])
        np.testing.assert_allclose(slam.to_map(bearing, 2), expected, atol=0.15)


@pytest.mark.parametrize("axis", [0, 1])
@pytest.mark.parametrize("sign", [-1, 1])
def test_translation_has_correct_world_axis_and_sign(axis, sign):
    slam = Slam2D()
    expected = np.zeros(3)
    for frame in range(8):
        expected[axis] = sign * frame * 0.1
        result = slam.update(*raycast(expected, frame=frame), t=frame * 0.1)
        assert result["ok"], (frame, result)
    predicted = result_pose(result)
    assert np.linalg.norm(predicted[:2] - expected[:2]) < 0.12
    assert angle_error(predicted[2], 0) < math.radians(2)


@pytest.mark.parametrize("sign", [-1, 1])
def test_turning_uses_clockwise_radians_from_positive_y(sign):
    slam = Slam2D()
    for frame in range(13):
        heading = math.radians(sign * frame * 5)
        result = slam.update(*raycast((0, 0, heading), frame=frame), t=frame * 0.1)
        assert result["ok"], (frame, result)
    predicted = result_pose(result)
    assert np.linalg.norm(predicted[:2]) < 0.15
    assert angle_error(predicted[2], heading) < math.radians(3)


@pytest.mark.parametrize("raw,dropped,sample_period", [(False, False, 0.1), (True, True, 0.1), (True, False, 0.25)])
def test_walking_turns_and_dropped_raw_acquisitions(raw, dropped, sample_period):
    slam, rows = run_trajectory(raw=raw, dropped=dropped, sample_period=sample_period)
    failures = [row for row in rows if not row["ok"]]
    assert not failures, failures[:3]
    errors = np.array([row["translation_error_m"] for row in rows])
    headings = np.array([row["heading_error_deg"] for row in rows])
    assert np.sqrt(np.mean(errors ** 2)) < 0.20, rows[int(np.argmax(errors))]
    assert errors.max() < 0.35, rows[int(np.argmax(errors))]
    assert headings.max() < 4.0, rows[int(np.argmax(headings))]
    assert len(slam.path) == len(rows)


def test_invalid_scan_rejection_preserves_pose_path_and_grid():
    slam = Slam2D()
    assert slam.update(*raycast(), t=1.0)["ok"]
    before = snapshot(slam)
    angles = np.arange(360, dtype=float)
    # Every entry is nonfinite, below the configured minimum, or above maximum.
    distances = np.resize([0, -100, 249, 12001, np.inf, np.nan], 360)
    result = slam.update(angles, distances, t=1.1)
    assert not result["ok"]
    result_pose(result)
    assert_unchanged(slam, before)
    result = slam.update(*raycast((0, 0.2, 0)), t=1.2)
    assert result["ok"]
    assert np.linalg.norm(result_pose(result)[:2] - [0, 0.2]) < 0.1


def test_invalid_samples_do_not_poison_an_otherwise_usable_raw_scan():
    slam = Slam2D()
    for frame in range(5):
        angles, distances = raycast((0.1 * frame, 0, 0), raw=True, frame=frame)
        distances[::17] = 0
        distances[3::29] = np.nan
        distances[5::41] = 30000
        angles[7::47] = np.nan
        result = slam.update(angles, distances, t=frame * 0.1)
        assert result["ok"], (frame, result)
    assert np.linalg.norm(result_pose(result)[:2] - [0.4, 0]) < 0.15


def test_nonmatching_scan_rejection_preserves_state():
    slam = Slam2D()
    assert slam.update(*raycast(), t=1.0)["ok"]
    before = snapshot(slam)
    # A close circular enclosure cannot match the initialized distant room.
    result = slam.update(np.arange(360), np.full(360, 1000.0), t=1.1)
    assert not result["ok"], result
    assert result["reason"] == "poor_match"
    assert_unchanged(slam, before)
    result = slam.update(*raycast((0.1, 0.1, 0)), t=1.2)
    assert result["ok"]
    assert np.linalg.norm(result_pose(result)[:2] - [0.1, 0.1]) < 0.1


@pytest.mark.parametrize("bootstrap", [False, True])
def test_tilt_is_rejected_before_any_integration(bootstrap):
    slam = Slam2D()
    if bootstrap:
        assert slam.update(*raycast(), t=1.0, acc=LEVEL_ACC)["ok"]
    before = snapshot(slam)
    radians = math.radians(15)
    rotation = np.array([[1, 0, 0], [0, math.cos(radians), -math.sin(radians)], [0, math.sin(radians), math.cos(radians)]])
    result = slam.update(*raycast((0, 0.1, 0)), t=1.1, acc=rotation @ LEVEL_ACC)
    assert not result["ok"], result
    assert_unchanged(slam, before)


@pytest.mark.parametrize("degrees,accepted", [(9.9, True), (10.1, False)])
def test_tilt_threshold_is_relative_to_calibrated_gravity(degrees, accepted):
    slam = Slam2D()
    assert slam.update(*raycast(), t=0, acc=LEVEL_ACC)["ok"]
    before = snapshot(slam)
    gravity = LEVEL_ACC / np.linalg.norm(LEVEL_ACC)
    transverse = np.cross(gravity, [1.0, 0.0, 0.0])
    transverse /= np.linalg.norm(transverse)
    # Construct exactly the requested angle to the measured reference vector.
    angle = math.radians(degrees)
    acceleration = np.linalg.norm(LEVEL_ACC) * (math.cos(angle) * gravity + math.sin(angle) * transverse)
    result = slam.update(*raycast(), t=0.1, acc=acceleration)
    assert bool(result["ok"]) == accepted, result
    if not accepted:
        assert_unchanged(slam, before)


def test_rejected_counter_counts_only_rejected_updates():
    slam = Slam2D()
    assert slam.rejected == 0
    assert not slam.update([], [], t=0)["ok"]
    assert slam.rejected == 1
    assert slam.update(*raycast(), t=1)["ok"]
    assert slam.rejected == 1
    assert not slam.update(*raycast(), t=2, acc=[0, 21.33, 0])["ok"]
    assert slam.rejected == 2
    assert not slam.update(np.arange(360), np.full(360, 500), t=3)["ok"]
    assert slam.rejected == 3
    assert slam.update(*raycast(), t=4)["ok"]
    assert slam.rejected == 3
    assert len(slam.path) == 2


def benchmark_report():
    # Warm the Python/NumPy execution paths outside the measured trajectory.
    warm = Slam2D()
    warm.update(*raycast())
    report = {}
    for raw, dropped, sample_period in [(False, False, 0.1), (True, True, 0.1), (True, False, 0.25)]:
        _, rows = run_trajectory(raw=raw, dropped=dropped, sample_period=sample_period)
        latency = np.array([row["latency_ms"] for row in rows[1:]])
        error = np.array([row["translation_error_m"] for row in rows])
        report[f"{'raw517' if raw else 'bins360'}_{round(1/sample_period)}hz{'_dropped' if dropped else ''}"] = {
            "observations": len(rows), "accepted": sum(row["ok"] for row in rows),
            "latency_p50_ms": float(np.median(latency)), "latency_p95_ms": float(np.percentile(latency, 95)),
            "rmse_m": float(np.sqrt(np.mean(error ** 2))), "max_error_m": float(error.max()),
            "max_heading_error_deg": max(row["heading_error_deg"] for row in rows),
            "rows": rows,
        }
    return report


if __name__ == "__main__":
    print(json.dumps(benchmark_report(), indent=2))
