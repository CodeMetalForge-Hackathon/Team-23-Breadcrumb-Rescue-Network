"""Mapper robustness beyond the independent trajectory acceptance tests."""

import math

import numpy as np
import pytest

from mapping import Slam2D


def room_scan(pose=(0., 0., 0.), n=360, noise_seed=None):
    """Analytic intersections with the four walls of an unequal rectangle."""
    angles = np.arange(n) * (360. / n)
    direction = np.deg2rad(angles) + pose[2]
    dx, dy = np.sin(direction), np.cos(direction)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx = np.where(dx >= 0, (5. - pose[0]) / dx, (-3. - pose[0]) / dx)
        ty = np.where(dy >= 0, (7. - pose[1]) / dy, (-4. - pose[1]) / dy)
    distances = np.minimum(tx, ty) * 1000.
    if noise_seed is not None:
        distances += np.random.default_rng(noise_seed).normal(0., 8., n)
    return angles, distances


@pytest.mark.parametrize("angles,ranges", [
    ([], []),
    (None, None),
    ([1, 2], [1000]),
    ([[0, 1]], [[1000, 1000]]),
    (["bad"], [1000]),
    ([0] * 360, [1000] * 360),
    (np.arange(360), np.zeros(360)),
])
def test_malformed_and_empty_scans_are_counted_without_initializing(angles, ranges):
    slam = Slam2D()
    result = slam.update(angles, ranges, t=0.)
    assert not result["ok"]
    assert slam.rejected == 1
    assert slam.pose == (0., 0., 0.)
    assert slam.path == []
    assert np.all(slam.grid()[0] == 127)
    assert all(math.isfinite(result[key]) for key in ("x", "y", "th", "score"))


@pytest.mark.parametrize("acc", [
    (0, 0, 0), (1, 2), (np.nan, 0, 21), (0, np.inf, 21),
    (0, 0, -21), (0, 0, 1), (0, 0, 100), "invalid",
])
def test_unreliable_acceleration_cannot_seed_map(acc):
    slam = Slam2D()
    result = slam.update(*room_scan(), t=0., acc=acc)
    assert not result["ok"]
    assert slam.rejected == 1
    assert slam.path == []
    assert np.all(slam.grid()[0] == 127)
    assert slam.update(*room_scan(), t=0., acc=None)["ok"]
    assert slam.rejected == 1


def test_rejections_do_not_change_accepted_clock_prediction_or_map():
    uninterrupted, interrupted = Slam2D(), Slam2D()
    for slam in (uninterrupted, interrupted):
        assert slam.update(*room_scan(), t=0.)["ok"]
        assert slam.update(*room_scan((.05, .08, .04)), t=.1)["ok"]
    bad_calls = [
        lambda: interrupted.update([], [], t=10.),
        lambda: interrupted.update(*room_scan(), t=.15, acc=(0, 0, 0)),
        lambda: interrupted.update(*room_scan(), t=.05),
        lambda: interrupted.update(*room_scan(), t=float("nan")),
        lambda: interrupted.update(np.arange(360), np.full(360, 700.), t=1.),
    ]
    before = interrupted.grid()[0]
    for call in bad_calls:
        assert not call()["ok"]
        assert interrupted.pose == uninterrupted.pose
        assert interrupted.path == uninterrupted.path
        np.testing.assert_array_equal(interrupted.grid()[0], before)
    a = uninterrupted.update(*room_scan((.1, .16, .08)), t=.2)
    b = interrupted.update(*room_scan((.1, .16, .08)), t=.2)
    assert a["ok"] and b["ok"]
    assert a == b
    np.testing.assert_array_equal(interrupted.grid()[0], uninterrupted.grid()[0])
    assert interrupted.rejected == len(bad_calls)


def test_collinear_returns_do_not_initialize_an_unobservable_scan():
    angles = np.linspace(-20., 20., 100)
    distances = 3000. / np.cos(np.deg2rad(angles))
    slam = Slam2D()
    result = slam.update(angles, distances)
    assert not result["ok"]
    assert result["reason"] == "degenerate_scan"
    assert slam.path == []


def test_path_and_grid_are_snapshots_not_writable_internal_state():
    slam = Slam2D()
    assert slam.update(*room_scan(), t=7.)["ok"]
    grid, origin, resolution = slam.grid()
    expected = grid.copy()
    grid[:] = 255
    path = slam.path
    path.clear()
    assert len(slam.path) == 1
    actual, actual_origin, actual_resolution = slam.grid()
    np.testing.assert_array_equal(actual, expected)
    assert actual_origin == origin
    assert actual_resolution == resolution


def test_broadcast_person_projection_and_invalid_person_observations():
    slam = Slam2D()
    x, y = slam.to_map([0., 90., 180., 270.], 2.)
    np.testing.assert_allclose(x, [0., 2., 0., -2.], atol=1e-12)
    np.testing.assert_allclose(y, [2., 0., -2., 0.], atol=1e-12)
    for bearing, distance in [(0., -1.), (np.nan, 1.), (1., np.inf)]:
        with pytest.raises(ValueError):
            slam.to_map(bearing, distance)


def test_timestamp_free_stream_accepts_motion_and_records_finite_times():
    slam = Slam2D()
    for i in range(8):
        result = slam.update(*room_scan((0., .1 * i, .015 * i)))
        assert result["ok"], result
    assert np.linalg.norm(np.array(slam.pose[:2]) - [0., .7]) < .12
    assert len(slam.path) == 8
    assert all(math.isfinite(row[0]) for row in slam.path)


def test_one_second_gap_is_recovered_against_accumulated_map():
    slam = Slam2D()
    assert slam.update(*room_scan(), t=0.)["ok"]
    expected = (.45, .75, math.radians(32))
    result = slam.update(*room_scan(expected, n=517), t=1.)
    assert result["ok"], result
    assert np.linalg.norm(np.asarray(slam.pose[:2]) - expected[:2]) < .12
    assert abs(slam.pose[2] - expected[2]) < math.radians(2)


def test_stationary_raw_noise_does_not_walk_the_map():
    slam = Slam2D()
    for i in range(80):
        result = slam.update(*room_scan(n=517, noise_seed=i), t=.1 * i)
        assert result["ok"], (i, result)
    assert np.linalg.norm(slam.pose[:2]) < .10
    assert abs(slam.pose[2]) < math.radians(1.)
    assert len(slam.path) == 80


@pytest.mark.parametrize("kwargs", [
    {"res": 0}, {"res": float("nan")}, {"size_m": -1},
    {"res": .00001}, {"min_range": 3, "max_range": 2},
])
def test_invalid_map_configuration_is_rejected_before_allocation(kwargs):
    with pytest.raises(ValueError):
        Slam2D(**kwargs)
