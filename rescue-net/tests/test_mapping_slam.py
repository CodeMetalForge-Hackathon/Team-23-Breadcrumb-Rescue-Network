"""Mapper robustness beyond the independent trajectory acceptance tests."""

import math
import threading
from concurrent.futures import ThreadPoolExecutor

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


def _map_patch(slam, x, y):
    grid, origin, resolution = slam.grid()
    ix, iy = np.floor((np.array([x, y]) - origin) / resolution).astype(int)
    return grid[iy - 1:iy + 2, ix - 1:ix + 2]


def _tilted_acc(degrees):
    gravity = np.array([1.45, 1.33, 21.26])
    sideways = np.cross(gravity, [1., 0., 0.])
    sideways *= np.linalg.norm(gravity) / np.linalg.norm(sideways)
    angle = math.radians(degrees)
    return math.cos(angle) * gravity + math.sin(angle) * sideways


def test_floor_risk_points_never_enter_map_but_near_walls_are_kept():
    angles = np.arange(360)
    distances = np.where(angles < 180, 2000., 9000.)
    slam = Slam2D(sensor_height_m=1., mount_tilt_deg=6.)
    assert slam.update(angles, distances, acc=_tilted_acc(0.))["ok"]
    assert 255 in _map_patch(slam, 2., 0.)
    assert 0 in _map_patch(slam, 1., 0.)
    # The suspicious far return is neither an obstacle nor a clearing ray.
    assert np.all(_map_patch(slam, -9., 0.) == 127)
    assert np.all(_map_patch(slam, -5., 0.) == 127)


def test_carried_rig_near_range_default_is_explicitly_overridable():
    angles = np.arange(360)
    distances = np.where(angles < 180, 400., 2000.)
    carried = Slam2D()
    unobstructed = Slam2D(min_range=.25)
    assert carried.update(angles, distances)["ok"]
    assert unobstructed.update(angles, distances)["ok"]
    assert np.all(_map_patch(carried, .4, 0.) == 127)
    assert 255 in _map_patch(unobstructed, .4, 0.)
    assert 255 in _map_patch(carried, -2., 0.)


def test_valid_rig_tilt_tightens_floor_bound_without_rejecting_good_near_scan():
    angles = np.arange(360)
    distances = np.where(angles < 180, 2000., 4500.)
    level, tilted = Slam2D(), Slam2D()
    assert level.update(angles, distances, acc=_tilted_acc(0.))["ok"]
    assert tilted.update(angles, distances, acc=_tilted_acc(9.))["ok"]
    assert 255 in _map_patch(level, -4.5, 0.)
    assert np.all(_map_patch(tilted, -4.5, 0.) == 127)
    assert 255 in _map_patch(tilted, 2., 0.)
    assert tilted.rejected == 0


def test_no_acc_or_disabled_floor_bound_preserves_configured_range():
    angles = np.arange(360)
    distances = np.where(angles < 180, 2000., 9000.)
    missing_acc = Slam2D()
    disabled = Slam2D(sensor_height_m=None)
    assert missing_acc.update(angles, distances)["ok"]
    assert disabled.update(angles, distances, acc=_tilted_acc(0.))["ok"]
    for slam in (missing_acc, disabled):
        assert 255 in _map_patch(slam, -9., 0.)
        assert slam.max_range == 12.  # Per-scan limits never mutate configuration.
    np.testing.assert_array_equal(missing_acc.grid()[0], disabled.grid()[0])


def test_floor_filter_does_not_bypass_whole_scan_tilt_gate():
    slam = Slam2D(sensor_height_m=1., mount_tilt_deg=6.)
    assert slam.update(*room_scan(), t=0., acc=_tilted_acc(0.))["ok"]
    before = slam.grid()[0]
    result = slam.update(*room_scan(), t=.1, acc=_tilted_acc(10.1))
    assert not result["ok"] and result["reason"] == "tilted"
    assert slam.path == [(0., 0., 0., 0.)]
    np.testing.assert_array_equal(slam.grid()[0], before)


@pytest.mark.parametrize("kwargs", [
    {"sensor_height_m": 0}, {"sensor_height_m": -1},
    {"sensor_height_m": float("inf")}, {"sensor_height_m": "bad"},
    {"mount_tilt_deg": -1}, {"mount_tilt_deg": 90},
    {"mount_tilt_deg": float("nan")},
])
def test_invalid_floor_calibration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        Slam2D(**kwargs)


def test_pause_before_seed_rejects_even_malformed_input_without_mutation():
    slam = Slam2D()
    assert slam.mapping_enabled is True
    slam.set_mapping_enabled(False)
    assert slam.mapping_enabled is False
    for angles, distances, stamp, acc in [
        (*room_scan(), 0., None),
        ([], [], float("nan"), "invalid"),
        (None, None, "bad", (0, 0, 0)),
    ]:
        result = slam.update(angles, distances, t=stamp, acc=acc)
        assert result == dict(x=0., y=0., th=0., ok=False, score=0.,
                              reason="mapping_paused")
    assert slam.rejected == 3
    assert slam.path == []
    assert np.all(slam.grid()[0] == 127)
    assert np.count_nonzero(slam._field) == 0
    assert slam._last_explicit_time is None
    slam.set_mapping_enabled(True)
    assert slam.update(*room_scan(), t=0.)["reason"] == "seed"
    assert slam.rejected == 3


def test_pause_preserves_accepted_state_and_resume_restarts_local_prediction(monkeypatch):
    slam = Slam2D()
    current_pose = (.08, .10, .03)
    assert slam.update(*room_scan(), t=100.)["ok"]
    assert slam.update(*room_scan(current_pose), t=100.1)["ok"]
    assert np.linalg.norm(slam._velocity) > .1
    pose, path = slam.pose, slam.path
    evidence, field = slam._evidence.copy(), slam._field.copy()
    velocity, stamp = slam._velocity.copy(), slam._last_explicit_time
    slam.set_mapping_enabled(False)
    for scan, t in [(room_scan((3., 2., 1.)), 500.), (([], []), -1.)]:
        assert slam.update(*scan, t=t)["reason"] == "mapping_paused"
        assert slam.pose == pose and slam.path == path
        assert slam._last_explicit_time == stamp
        np.testing.assert_array_equal(slam._velocity, velocity)
        np.testing.assert_array_equal(slam._evidence, evidence)
        np.testing.assert_array_equal(slam._field, field)
    slam.set_mapping_enabled(True)
    assert slam.mapping_enabled is True
    assert slam.pose == pose and slam.path == path
    np.testing.assert_array_equal(slam._evidence, evidence)
    np.testing.assert_array_equal(slam._field, field)
    assert slam._last_explicit_time == stamp
    np.testing.assert_array_equal(slam._velocity, 0.)
    # Invalid and stale scans must neither advance the clock nor consume the
    # one-shot nominal search reserved for the next accepted resumed scan.
    assert not slam.update([], [], t=1000.)["ok"]
    for stale in (0., stamp):
        result = slam.update(*room_scan(current_pose), t=stale)
        assert result["reason"] == "non_monotonic_time"
    assert slam._last_explicit_time == stamp
    assert slam._resume_pending
    assert slam.pose == pose and slam.path == path
    np.testing.assert_array_equal(slam._evidence, evidence)
    np.testing.assert_array_equal(slam._field, field)
    original_match = slam._match
    matched = []

    def capture_match(points, dt):
        matched.append((dt, slam._velocity.copy()))
        return original_match(points, dt)

    monkeypatch.setattr(slam, "_match", capture_match)
    result = slam.update(*room_scan(current_pose), t=200.)
    assert result["ok"] and result["reason"] == "matched"
    assert len(matched) == 1 and matched[0][0] == .1
    np.testing.assert_array_equal(matched[0][1], 0.)
    assert np.linalg.norm(np.asarray(slam.pose[:2]) - current_pose[:2]) < .1
    assert slam.path[:-1] == path and slam.path[-1][0] == 200.
    assert not slam._resume_pending
    assert slam.rejected == 5
    assert slam.update(*room_scan(current_pose), t=200.)["reason"] == "non_monotonic_time"
    assert slam.update(*room_scan(current_pose), t=200.3)["ok"]
    assert len(matched) == 2 and matched[1][0] == pytest.approx(.3)


def test_repeated_enable_does_not_reset_an_active_stream():
    slam = Slam2D()
    assert slam.update(*room_scan(), t=10.)["ok"]
    assert slam.update(*room_scan((.05, .08, .02)), t=10.1)["ok"]
    before = slam._velocity.copy()
    slam.set_mapping_enabled(True)
    np.testing.assert_array_equal(slam._velocity, before)
    assert slam.update(*room_scan(), t=0.)["reason"] == "non_monotonic_time"
    assert len(slam.path) == 2


@pytest.mark.parametrize("value", [None, 0, 1, "false", [], float("nan")])
def test_pause_control_rejects_nonboolean_values_without_changing_state(value):
    slam = Slam2D()
    with pytest.raises(TypeError, match="boolean"):
        slam.set_mapping_enabled(value)
    assert slam.mapping_enabled is True
    assert slam.rejected == 0
    assert slam.path == []


def test_pause_waits_for_inflight_integration_and_blocks_later_updates(monkeypatch):
    slam = Slam2D()
    integrating, release = threading.Event(), threading.Event()
    pausing = threading.Event()
    original_integrate = slam._integrate

    def blocked_integrate(points, pose):
        integrating.set()
        assert release.wait(2.), "test did not release in-flight update"
        original_integrate(points, pose)

    def pause():
        pausing.set()
        slam.set_mapping_enabled(False)

    monkeypatch.setattr(slam, "_integrate", blocked_integrate)
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending_update = pool.submit(slam.update, *room_scan(), t=0.)
        try:
            assert integrating.wait(2.)
            pending_pause = pool.submit(pause)
            assert pausing.wait(2.)
            assert not pending_pause.done()
        finally:
            release.set()
        assert pending_update.result(timeout=2.)["ok"]
        pending_pause.result(timeout=2.)
    assert slam.mapping_enabled is False
    assert len(slam.path) == 1
    before = slam.grid()[0]
    assert slam.update(*room_scan((1., 0., 0.)), t=1.)["reason"] == "mapping_paused"
    assert len(slam.path) == 1 and slam.rejected == 1
    np.testing.assert_array_equal(slam.grid()[0], before)
