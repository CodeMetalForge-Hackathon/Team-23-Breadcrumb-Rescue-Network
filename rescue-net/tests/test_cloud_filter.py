"""Surface-filter geometry: retain obstacles and avoid inventing flat planes."""

import base64
import math

import numpy as np
import pytest

from dashboard.cloud import Cloud, LEVEL_REFERENCE, _classify_surfaces, _horizontal_levels


def flags(snapshot):
    out = np.frombuffer(base64.b64decode(snapshot["surface_flags"]), dtype=np.uint8)
    assert len(out) == snapshot["count"]
    assert np.count_nonzero(out) == snapshot["hidden_count"]
    assert np.count_nonzero(out == 0) == snapshot["visible_count"]
    return out


def plane(z, half=3.0, step=.15):
    x, y = np.meshgrid(np.arange(-half, half + step / 2, step),
                       np.arange(-half, half + step / 2, step))
    # Slight unevenness exercises histogram bin boundaries.
    height = z + .005 * np.sin(x * 3) * np.cos(y * 2)
    return np.column_stack((x.ravel(), y.ravel(), height.ravel()))


def test_conservative_range_hides_long_returns_but_keeps_close_obstacles():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [0, 90, 180, 270], [2000, 9000, 400, 5000], LEVEL_REFERENCE, True)
    snapshot = cloud.snapshot()
    np.testing.assert_array_equal(flags(snapshot), [0, 1, 0, 0])
    assert snapshot["count"] == 4 and snapshot["visible_count"] == 3
    assert snapshot["surface_filter"]["floor_z"] is None
    assert snapshot["surface_filter"]["ceiling_z"] is None
    assert snapshot["filtered_bounds"] != snapshot["bounds"]


def test_missing_acceleration_keeps_flat_cloud_without_false_floor():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), np.arange(360), np.full(360, 9000), None, True)
    np.testing.assert_array_equal(flags(cloud.snapshot()), 0)
    assert cloud.snapshot()["visible_count"] == cloud.snapshot()["count"]


def test_more_tilt_shortens_the_safe_range_on_both_sides():
    angle = math.radians(5)
    rotation = np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0],
                         [-math.sin(angle), 0, math.cos(angle)]])
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [90, 270, 0], [5000, 5000, 2000], rotation @ LEVEL_REFERENCE, True)
    np.testing.assert_array_equal(flags(cloud.snapshot()), [1, 1, 0])


def test_a_close_observation_protects_the_same_wall_voxel():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [0], [9000], LEVEL_REFERENCE, True)
    assert flags(cloud.snapshot())[0] == 1
    cloud.add_scan((0, 7, 0), [0], [2000], LEVEL_REFERENCE, True)
    snapshot = cloud.snapshot()
    assert snapshot["count"] == 1
    assert flags(snapshot)[0] == 0


def test_explicit_mount_height_can_be_changed_or_range_gate_disabled():
    low = Cloud(sensor_height_m=.25)
    low.add_scan((0, 0, 0), [0, 90], [1000, 3000], LEVEL_REFERENCE, True)
    np.testing.assert_array_equal(flags(low.snapshot()), [0, 1])
    disabled = Cloud(sensor_height_m=None)
    disabled.add_scan((0, 0, 0), [0, 90], [1000, 3000], LEVEL_REFERENCE, True)
    np.testing.assert_array_equal(flags(disabled.snapshot()), [0, 0])


def test_dense_floor_ceiling_and_outside_points_are_hidden_obstacles_survive():
    floor, ceiling = plane(-1), plane(1.8)
    rng = np.random.default_rng(9)
    obstacles = np.column_stack((rng.uniform(-2, 2, 600), rng.uniform(-2, 2, 600),
                                 rng.uniform(-.7, 1.45, 600)))
    outside = np.array([[0, 0, -1.6], [0, 0, 2.4]])
    points = np.vstack((floor, ceiling, obstacles, outside))
    classified, low, high, height = _classify_surfaces(points, np.zeros(len(points)), 1.0, .15)
    assert low == pytest.approx(-1, abs=.02) and high == pytest.approx(1.8, abs=.02)
    assert height == pytest.approx(1)
    assert np.all(classified[:len(floor)] & 2)
    assert np.all(classified[len(floor):len(floor) + len(ceiling)] & 4)
    np.testing.assert_array_equal(classified[len(floor) + len(ceiling):-2], 0)
    assert classified[-2] & 2 and classified[-1] & 4


def test_low_rig_floor_is_learned_without_assuming_carried_height():
    floor, ceiling = plane(-.25), plane(2.5)
    obstacles = np.array([[0, 0, -.04], [.3, .1, .2], [1, .5, .8]])
    points = np.vstack((floor, ceiling, obstacles))
    classified, low, high, height = _classify_surfaces(points, np.zeros(len(points)), 1.0, .15)
    assert low == pytest.approx(-.25, abs=.02)
    assert high == pytest.approx(2.5, abs=.02)
    assert height == pytest.approx(.25, abs=.02)
    np.testing.assert_array_equal(classified[-3:], 0)


def test_horizontal_bands_on_vertical_walls_are_not_floor_or_ceiling():
    # A four-metre room perimeter has a large footprint but no interior XY
    # support. Height peaks along it must not erase the real walls.
    t = np.linspace(-2, 2, 150)
    perimeter = np.vstack((np.column_stack((t, np.full_like(t, -2))),
                           np.column_stack((t, np.full_like(t, 2))),
                           np.column_stack((np.full_like(t, -2), t)),
                           np.column_stack((np.full_like(t, 2), t))))
    points = np.vstack([np.column_stack((perimeter, np.full(len(perimeter), z)))
                        for z in [-1, 0, 1.8]])
    assert _horizontal_levels(points) == (None, None)
    classified, *_ = _classify_surfaces(points, np.zeros(len(points)), None, .15)
    np.testing.assert_array_equal(classified, 0)


def test_small_tabletop_and_flat_scanner_band_are_not_room_boundaries():
    table = plane(-.3, half=.6, step=.05)
    rng = np.random.default_rng(20)
    background = rng.uniform([-3, -3, -.9], [3, 3, 1.6], (1000, 3))
    assert _horizontal_levels(np.vstack((table, background))) == (None, None)
    assert _horizontal_levels(plane(0)) == (None, None)


def test_surface_state_clears_with_map_and_rejected_scan_keeps_mask():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [0], [9000], LEVEL_REFERENCE, True)
    before = cloud.snapshot()
    cloud.add_scan((1, 2, 0), [0], [1000], LEVEL_REFERENCE, False)
    assert cloud.snapshot()["surface_flags"] == before["surface_flags"]
    cloud.clear()
    empty = cloud.snapshot()
    assert empty["surface_flags"] == "" and empty["hidden_count"] == 0
    assert empty["visible_count"] == 0 and empty["filtered_bounds"] is None
    assert empty["surface_filter"]["floor_z"] is None


@pytest.mark.parametrize("kwargs", [{"sensor_height_m": 0}, {"sensor_height_m": float("nan")},
                                    {"mount_tilt_deg": -1}, {"mount_tilt_deg": 90},
                                    {"plane_margin_m": 0}, {"plane_margin_m": .6}])
def test_invalid_filter_settings_fail_early(kwargs):
    with pytest.raises(ValueError):
        Cloud(**kwargs)
