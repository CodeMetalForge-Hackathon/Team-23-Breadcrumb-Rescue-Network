import gzip
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

from mapping.relay import (MODEL, RelayModel, count_wall_crossings,
                           count_wall_segments, load_model, suggest_relays)


def corridor():
    return np.zeros((20, 200), dtype=np.uint8), (-1.0, -2.0), 0.25


def test_walls_are_contiguous_segments_not_pixels():
    grid = np.zeros((8, 40), dtype=np.uint8)
    grid[:, 7:15] = 255
    grid[:, 24:27] = 255
    for start, end in (((1.5, 3.5), (35.5, 3.5)), ((35.5, 3.5), (1.5, 3.5))):
        assert count_wall_segments(grid, (0, 0), 1, start, end) == 2
    assert count_wall_segments(grid, (0, 0), 1, (8.5, 3.5), (13.5, 3.5)) == 1
    assert count_wall_segments(grid, (0, 0), 1, (1.5, 3.5), (6.5, 3.5)) == 0


def test_diagonal_ray_and_outside_map_do_not_wrap_indices():
    grid = np.zeros((12, 12), dtype=np.uint8)
    grid[5:8, :] = 255
    assert count_wall_segments(grid, (0, 0), 1, (0.5, 0.5), (11.5, 11.5)) == 1
    assert count_wall_segments(grid, (0, 0), 1, (-3.5, 4.5), (-0.5, 8.5)) == 0


def test_relays_precede_distance_threshold_and_are_free():
    grid, origin, res = corridor()
    path = [(float(x), float(x), 0.0, 0.0) for x in range(41)]
    recommendations = suggest_relays(grid, origin, res, {"base": (0.0, 0.0)}, path)
    assert len(recommendations) >= 3
    anchor = (0.0, 0.0)
    maximum_distance = 10 ** ((MODEL.reference_margin_db - MODEL.trigger_margin_db)
                              / MODEL.distance_loss_db)
    for item in recommendations:
        x, y, reason = item["x"], item["y"], item["reason"]
        assert MODEL.min_spacing_m <= math.dist(anchor, (x, y)) <= maximum_distance
        ix, iy = int((x - origin[0]) / res), int((y - origin[1]) / res)
        assert grid[iy, ix] == 0
        assert "Heuristic, unmeasured" in reason
        anchor = (x, y)
    assert recommendations[0]["x"] < maximum_distance


def test_reached_existing_node_resets_the_link_anchor():
    grid, origin, res = corridor()
    path = [(float(x), 0.0) for x in range(31)]
    recommendations = suggest_relays(grid, origin, res,
                                     {"base": (0.0, 0.0), "relay": (10.0, 0.0)}, path)
    assert recommendations
    assert recommendations[0]["x"] > 20.0
    assert suggest_relays(grid, origin, res, {"base": (0.0, 0.0)},
                          [(i, x, y, 0.0) for i, (x, y) in enumerate(path)]) == suggest_relays(
                              grid, origin, res, {"base": (0.0, 0.0)}, path)


def test_retracing_path_reuses_suggested_nodes_without_duplicate_placements():
    grid, origin, res = corridor()
    outward = [(float(x), 0.0) for x in range(39)]
    loop = outward + outward[-2::-1] + outward[1:]
    nodes = {"base": (0.0, 0.0)}
    once = suggest_relays(grid, origin, res, nodes, outward)
    repeated = suggest_relays(grid, origin, res, nodes, loop)
    assert repeated == once
    assert nodes == {"base": (0.0, 0.0)}


def test_no_placement_on_unknown_occupied_or_map_exterior():
    path = [(float(x), 0.0) for x in range(31)]
    for value in (127, 255):
        grid, origin, res = corridor()
        grid[:] = value
        assert suggest_relays(grid, origin, res, {"base": (0.0, 0.0)}, path) == []
    grid, origin, res = corridor()
    assert suggest_relays(grid, origin, res, {"base": (0.0, -10.0)},
                          [(x, -10.0) for x, _ in path]) == []


def test_wall_loss_triggers_earlier_placement():
    grid, origin, res = corridor()
    grid[:, 32:36] = 255  # wall from x=7 to x=8 m
    result = suggest_relays(grid, origin, res, {"base": (0.0, 0.0)}, [(0, 0), (14, 0)])
    assert result
    assert result[0]["x"] < 7.0
    assert "1 wall segment(s)" in result[0]["reason"]


def test_unknown_radio_path_is_explicit_in_reason():
    grid, origin, res = corridor()
    grid[:, 16:20] = 127
    result = suggest_relays(grid, origin, res, {"base": (0.0, 0.0)}, [(0, 0), (14, 0)])
    assert result and "crosses unmapped space" in result[0]["reason"]


def test_invalid_grid_and_path_fail_clearly():
    grid, origin, res = corridor()
    with pytest.raises(ValueError, match="0.*127.*255"):
        suggest_relays(np.ones_like(grid), origin, res, {"base": (0, 0)}, [])
    assert suggest_relays(grid, origin, res, {}, []) == []
    with pytest.raises(ValueError, match="mapping"):
        suggest_relays(grid, origin, res, [], [])
    with pytest.raises(ValueError, match="path\\[0\\]"):
        suggest_relays(grid, origin, res, {"base": (0, 0)}, [(1, 2, 3)])
    with pytest.raises(ValueError, match="res"):
        suggest_relays(grid, origin, None, {"base": (0, 0)}, [])


@pytest.mark.parametrize("walls,expected", [
    ([], 0),
    ([[2, -2, 2, 2]], 1),
    ([[2, 2, 2, -2], [2.1, -1, 2.1, 1]], 1),
    ([[2, -2, 2, 2], [2.2, -2, 2.2, 2]], 2),
    ([[1, -2, 1, 2], [1.1, -2, 1.1, 2], [1.2, -2, 1.2, 2]], 2),
    ([[2, -2, 2, 0], [2, 0, 2, 2]], 1),
    ([[2, -2, 2, -.1], [2, .1, 2, 2]], 0),
    ([[2, -3, 2, -.2], [2, .2, 2, 3], [2, -.5, 2, .5]], 1),
    ([[2, -3, 2, 0], [2, 0, 2, 3], [2, -1, 2, 1]], 1),
    ([[0, 0, 4, 0]], 0),  # Collinear travel.
    ([[0, -2, 0, 2], [4, -2, 4, 2]], 0),  # Link endpoints.
    ([[2, 0, 2, 2]], 0),  # Isolated endpoint touch.
    ([[1, 1, 2, 0], [2, 0, 3, 1]], 0),  # Same-side corner tangency.
    ([[1, 1, 2, 0], [2, 0, 3, -1]], 1),  # Opposite-side corner passage.
    ([[1, -1, 3, 1], [1, 1, 3, -1]], 2),  # Nonparallel intersection.
    ([[2, -2, 2, 2], [1.98, -2, 2.02, 2]], 2),  # Shallow real intersection.
])
def test_geometric_wall_crossing_rules_are_symmetric(walls, expected):
    assert count_wall_crossings((0, 0), (4, 0), walls) == expected
    assert count_wall_crossings((4, 0), (0, 0), list(reversed(walls))) == expected


def test_explicit_empty_walls_ignore_same_room_grid_speckles_for_radio_model():
    grid, origin, res = corridor()
    clean = suggest_relays(grid, origin, res, {"base": (0, 0)}, [(0, 0), (14, 0)], walls=[])
    for column in (16, 24, 32):
        grid[8, column] = 255
    geometric = suggest_relays(grid, origin, res, {"base": (0, 0)}, [(0, 0), (14, 0)], walls=[])
    fallback = suggest_relays(grid, origin, res, {"base": (0, 0)}, [(0, 0), (14, 0)])
    assert geometric[0]["x"] == clean[0]["x"]
    assert fallback[0]["x"] < geometric[0]["x"]
    assert "0 wall segment(s)" in geometric[0]["reason"]


def test_explicit_wall_penalty_and_duplicate_copy_do_not_compound():
    grid, origin, res = corridor()
    args = grid, origin, res, {"base": (0, 0)}, [(0, 0), (14, 0)]
    one = suggest_relays(*args, walls=[[7, -2, 7, 2]])
    duplicate = suggest_relays(*args, walls=[[7, -2, 7, 2], [7, 2, 7, -2]])
    assert one == duplicate
    assert one[0]["x"] <= 7.
    assert "1 wall segment(s)" in one[0]["reason"]


def test_bad_measurement_overrides_optimistic_model_at_last_good_breadcrumb():
    grid, origin, res = corridor()
    result = suggest_relays(grid, origin, res, {"base": (0, 0)}, [(0, 0), (8, 0)],
                            walls=[], measured=[(5, 0, -85, "base")])
    assert len(result) == 1
    assert result[0]["x"] == 4.25  # At 4.5 m the bad observation is within 0.5 m.
    assert "uncalibrated" in result[0]["reason"]
    assert "measured RSSI from real node 'base' is -85.0 dBm" in result[0]["reason"]
    assert "here" in result[0]["reason"] and "model RSSI)" in result[0]["reason"]


def test_good_measurements_override_pessimistic_model_and_source_the_breadcrumb():
    grid, origin, res = corridor()
    args = grid, origin, res, {"base": (0, 0)}, [(0, 0), (16, 0)]
    default = suggest_relays(*args, walls=[])
    measured = [(x, 0, -60, "base") for x in np.arange(12.5, 15., .25)]
    result = suggest_relays(*args, walls=[], measured=measured)
    assert result[0]["x"] > default[0]["x"] + 2.
    assert "here -60.0 dBm, measured RSSI from real node 'base'" in result[0]["reason"]
    assert "uncalibrated" in result[0]["reason"]


def test_wrong_anchor_and_distant_measurements_do_not_override():
    grid, origin, res = corridor()
    args = grid, origin, res, {"base": (0, 0)}, [(0, 0), (14, 0)]
    expected = suggest_relays(*args, walls=[])
    for measured in ([(5, 0, -100, "other")], [(5, .501, -100, "base")]):
        assert suggest_relays(*args, walls=[], measured=measured) == expected


def test_measurements_are_not_reused_by_hypothetical_anchor_or_wrong_real_node():
    grid, origin, res = corridor()
    measured = [(x, 0, -100, "base") for x in np.arange(5., 40., .25)]
    result = suggest_relays(grid, origin, res, {"base": (0, 0)}, [(0, 0), (30, 0)],
                            walls=[], measured=measured)
    assert len(result) == 3
    assert result[0]["x"] == 4.25
    assert result[1]["x"] > 16.
    assert "measured RSSI" not in result[1]["reason"]
    result = suggest_relays(grid, origin, res, {"base": (0, 0), "relay": (10, 0)},
                            [(0, 0), (20, 0)], walls=[], measured=[(15, 0, -100, "base")])
    assert result == []


def test_closest_measurement_wins_and_latest_input_breaks_ties():
    grid, origin, res = corridor()
    args = grid, origin, res, {"base": (0, 0)}, [(0, 0), (8, 0)]
    assert suggest_relays(*args, walls=[], measured=[(5, 0, -85, "base"),
                                                    (5, 0, -60, "base")]) == []
    assert suggest_relays(*args, walls=[], measured=[(5, 0, -60, "base"),
                                                    (5, 0, -85, "base")])


def test_calibrated_model_loads_explicitly_and_ignores_fit_metadata(tmp_path):
    payload = dict(p0_dbm=-45, path_loss_exponent=2.5, wall_loss_db=4,
                   threshold_dbm=-72, calibrated=True,
                   diagnostics={"samples": 40}, fit={"coefficients": [-999, -999]})
    target = tmp_path / "model.json"
    target.write_text(json.dumps(payload), encoding="utf-8")
    model = load_model(target)
    assert model == RelayModel.from_dict(payload)
    assert model.p0_dbm == -45 and model.calibrated is True
    grid, origin, res = corridor()
    result = suggest_relays(grid, origin, res, {"base": (0, 0)}, [(0, 0), (20, 0)],
                            walls=[], model=model)
    assert result and "Calibrated model" in result[0]["reason"]
    assert "uncalibrated" not in result[0]["reason"]
    assert MODEL.calibrated is False and MODEL.p0_dbm == -50.


@pytest.mark.parametrize("kwargs", [
    {"p0_dbm": float("nan")}, {"threshold_dbm": float("inf")},
    {"path_loss_exponent": 0}, {"wall_loss_db": -1}, {"calibrated": "false"},
    {"path_step_m": 0}, {"min_spacing_m": -1}, {"placement_clearance_m": -1},
])
def test_invalid_model_constants_fail_clearly(kwargs):
    with pytest.raises(ValueError):
        RelayModel(**kwargs)


@pytest.mark.parametrize("walls", [[[0, 0, 0, 0]], [[0, 0, 1]],
                                   [[0, 0, float("nan"), 1]], None])
def test_invalid_geometric_walls_fail_clearly(walls):
    with pytest.raises(ValueError, match="walls"):
        count_wall_crossings((0, 0), (1, 0), walls)


@pytest.mark.parametrize("measured", [3, [[0, 0, -80]], [[0, 0, float("nan"), "base"]],
                                      [[0, 0, -80, None]]])
def test_invalid_measurements_fail_clearly(measured):
    grid, origin, res = corridor()
    with pytest.raises(ValueError, match="measured"):
        suggest_relays(grid, origin, res, {"base": (0, 0)}, [], measured=measured)


def test_model_loader_rejects_partial_or_invalid_json(tmp_path):
    with pytest.raises(ValueError, match="model must provide"):
        RelayModel.from_dict({"calibrated": True})
    path = tmp_path / "bad.json"
    path.write_text("not JSON", encoding="utf-8")
    with pytest.raises(ValueError):
        load_model(path)


# Fitter acceptance shares the public geometry/model contract used above.
_fit_spec = importlib.util.spec_from_file_location(
    "fit_relay_model_tested", Path(__file__).resolve().parents[2] / "tools" / "fit_relay_model.py")
_fit_tool = importlib.util.module_from_spec(_fit_spec)
_fit_spec.loader.exec_module(_fit_tool)

_FIT_WALLS = [[2.0, -3.0, 2.0, 3.0], [4.0, -3.0, 4.0, 3.0]]
_FIT_POINTS = [(1.0, 0.0), (1.0, 1.0), (1.5, 0.0), (2.5, 0.0), (3.0, 0.0), (3.0, 2.0), (5.0, 0.0), (5.0, 2.0), (7.0, 3.0), (8.0, 5.0), (1.0, 3.0), (3.0, 5.0)]

def _fit_geometry():
    distance = np.array([math.hypot(*p) for p in _FIT_POINTS])
    walls = np.array([count_wall_crossings((0, 0), p, _FIT_WALLS) for p in _FIT_POINTS])
    rssi = -45 - 23 * np.log10(np.maximum(distance, 1)) - 6.5 * walls
    return (distance, walls, rssi)

def test_fitter_known_geometry_recovers_three_coefficients_and_loads():
    result = _fit_tool.fit_samples(*_fit_geometry())
    assert result['calibrated']
    assert [result[k] for k in _fit_tool.COEFFICIENT_NAMES] == pytest.approx([-45, 2.3, 6.5])
    assert result['fit']['rank'] == 3
    assert result['fit']['rms_db'] < 1e-10
    assert all(result['fit']['identifiable'].values())
    assert RelayModel.from_dict(result).calibrated

@pytest.mark.parametrize('walls', [np.zeros(12), np.ones(12) * 2])
def test_fitter_constant_wall_count_keeps_safe_defaults_and_explains_confounding(walls):
    distance = np.geomspace(1, 15, 12)
    result = _fit_tool.fit_samples(distance, walls, -45 - 20 * np.log10(distance) - 8 * walls)
    assert not result['calibrated']
    assert result['wall_loss_db'] == 8
    assert not result['fit']['identifiable']['wall_loss_db']
    explanation = ' '.join(result['fit']['rejection_reasons'])
    assert ('unobserved' if walls[0] == 0 else 'confounds') in explanation
    assert RelayModel.from_dict(result).calibrated is False

def test_fitter_constant_distance_and_collinear_features_are_not_identifiable():
    walls = np.arange(12) % 3
    result = _fit_tool.fit_samples(np.ones(12) * 3, walls, -50 - 8 * walls)
    assert not result['calibrated']
    assert not result['fit']['identifiable']['path_loss_exponent']
    distance = 10 ** (0.3 * walls)
    result = _fit_tool.fit_samples(distance, walls, -45 - 20 * np.log10(distance) - 4 * walls)
    assert not result['calibrated']
    assert result['fit']['rank'] == 2
    assert not result['fit']['identifiable']['wall_loss_db']
    assert not result['fit']['identifiable']['path_loss_exponent']

def test_fitter_ill_conditioned_and_small_distance_variation_fail():
    walls = np.arange(12) % 3
    distance = 10 ** (0.3 * walls + 1e-07 * np.arange(12))
    result = _fit_tool.fit_samples(distance, walls, -45 - 20 * np.log10(distance) - 4 * walls)
    assert result['fit']['rank'] == 3
    assert result['fit']['condition_number'] > _fit_tool.MAX_CONDITION
    assert not result['calibrated']
    distance = 1 + np.arange(12) * 0.001
    result = _fit_tool.fit_samples(distance, walls, -45 - 20 * np.log10(distance) - 4 * walls)
    assert not result['calibrated']
    assert 'insufficient distance variation' in result['fit']['rejection_reasons']

@pytest.mark.parametrize('coefficients', [(-45, -1, 3), (-45, 2, -3), (-45, 8, 3), (-130, 0.5, 0)])
def test_fitter_nonphysical_coefficients_never_activate(coefficients):
    distance, walls, _ = _fit_geometry()
    p0, n, w = coefficients
    rssi = p0 - 10 * n * np.log10(distance) - w * walls
    result = _fit_tool.fit_samples(distance, walls, rssi)
    assert not result['calibrated']
    assert all((result[key] == value for key, value in _fit_tool.DEFAULT_MODEL.items()))
    assert any(('nonphysical' in reason for reason in result['fit']['rejection_reasons']))

@pytest.mark.parametrize('args', [([1, 2], [0], [-40, -50]), ([1, np.nan], [0, 1], [-40, -50]), ([1, 2], [0, 0.5], [-40, -50]), ([1, 2], [0, -1], [-40, -50]), ([1, 2], [0, 1], [True, False]), ([1, 2], [0, 1], [-151, -50]), ([-1, 2], [0, 1], [-40, -50])])
def test_fitter_invalid_fit_data(args):
    with pytest.raises(_fit_tool.ReplayError):
        _fit_tool.fit_samples(*args)

class _FitFakeSlam:

    def __init__(self, states):
        self.states = states
        self.seen = []

    def update(self, angles, mm, t, acc=None):
        self.seen.append((t, list(angles), list(mm), acc))
        state = self.states[t]
        return dict(x=state.get('x', 0), y=state.get('y', 0), th=0.0, ok=state.get('ok', True), score=state.get('score', 0.9), reason=state.get('reason', 'matched'))

    def grid(self):
        return (np.full((10, 10), 127, np.uint8), (-1.0, -1.0), 0.5)

def _fit_scan(t, rx):
    return dict(t=t, rx=rx, a=None, ang=[0, 9000], mm=[1000, 2000])

def _fit_meta(t, rx, rssi):
    return dict(rx=rx, m=dict(t=t, rssi=float(rssi)))

def _fit_recording(tmp_path, rows):
    path = tmp_path / 'known.jsonl.gz'
    with gzip.open(path, 'wt') as stream:
        stream.write(json.dumps({'meta': {}}) + '\n')
        for row in rows:
            stream.write(json.dumps(row) + '\n')
    return path

def _fit_known_recording(tmp_path):
    _, _, rssi = _fit_geometry()
    rows = [_fit_scan(0, 100), _fit_meta(0.1, 100.1, -20)]
    states = {0: dict(x=0, y=0)}
    for i, (point, signal) in enumerate(zip(_FIT_POINTS, rssi)):
        t = 2 + 3 * i
        states[t] = dict(x=point[0], y=point[1])
        rows.extend([_fit_scan(t, 100 + t + 0.2), _fit_meta(t + 0.02, 100 + t + 0.3, signal), _fit_meta(t + 0.3, 100 + t + 0.6, signal)])
    return (_fit_recording(tmp_path, rows), _FitFakeSlam(states))

def _fit_extract_known(*args):
    return dict(walls=_FIT_WALLS, angle=0.0, cells=20)

def test_fitter_recording_replay_units_dedup_pose_timestamps_and_window(tmp_path):
    path, slam = _fit_known_recording(tmp_path)
    result = _fit_tool.fit_recording(path, (0, 0), slam=slam, wall_extractor=_fit_extract_known, node_uncertainty_m=0.5)
    assert result['calibrated']
    assert result['fit']['sample_count'] == 12
    assert result['source']['counts']['duplicate_rssi_broadcasts'] == 12
    assert result['source']['counts']['initial_unknown_age_run'] == 1
    assert result['source']['node_uncertainty_m'] == 0.5
    first = result['samples'][0]
    assert (first['pose_t'], first['pose_rx'], first['meta_t'], first['meta_rx']) == (2, 102.2, 2.02, 102.3)
    assert first['pose_age_s'] == pytest.approx(0.02)
    assert first['alignment_clock'] == 'source_t'
    assert slam.seen[0][1:3] == ([0.0, 90.0], [1000.0, 2000.0])
    path, slam = _fit_known_recording(tmp_path)
    result = _fit_tool.fit_recording(path, (0, 0), t0=8.3, t1=26.3, slam=slam, wall_extractor=_fit_extract_known)
    assert len(slam.seen) == 13
    samples = result['samples']
    assert len(samples) == 7
    assert all(8.3 - _fit_tool.WINDOW_EPS_S <= q['elapsed_rx_s']
               <= 26.3 + _fit_tool.WINDOW_EPS_S for q in samples)
    assert samples[0]['elapsed_rx_s'] == pytest.approx(8.3)
    assert samples[-1]['elapsed_rx_s'] == pytest.approx(26.3)
    # Inclusive comparisons tolerate rounding; saved timestamps stay raw.
    assert samples[0]['meta_rx'] == 108.3
    assert samples[0]['elapsed_rx_s'] == 108.3 - 100.
    assert samples[-1]['meta_rx'] == 126.3
    assert samples[-1]['elapsed_rx_s'] == 126.3 - 100.
    path, slam = _fit_known_recording(tmp_path)
    outside = _fit_tool.fit_recording(path, (0, 0), t0=8.30001, t1=26.3,
                                      slam=slam, wall_extractor=_fit_extract_known)
    assert len(outside['samples']) == 6
    assert outside['samples'][0]['elapsed_rx_s'] == pytest.approx(11.3)

def test_fitter_no_future_source_pose_stale_and_rejected_poses_do_not_leak(tmp_path):
    states = {0: {}, 2: {}, 4: dict(ok=False), 6: dict(score=0.4), 8: {}, 10: {}, 12: {}}
    rows = [_fit_scan(0, 100), _fit_meta(0.1, 100.1, -30), _fit_scan(2, 102), _fit_meta(2.8, 102.8, -40), _fit_scan(4, 104), _fit_meta(4.9, 104.9, -41), _fit_scan(6, 106), _fit_scan(8, 108), _fit_meta(7.9, 109.1, -43), _fit_scan(10, 110), _fit_meta(10.1, 111.2, -44), _fit_scan(12, 112), dict(rx=113.3, m={'rssi': -45})]
    result = _fit_tool.fit_recording(_fit_recording(tmp_path, rows), (1, 0), slam=_FitFakeSlam(states), wall_extractor=_fit_extract_known)
    counts = result['source']['counts']
    assert counts['stale_pose'] == 2
    assert counts['poor_tracking'] >= 2
    assert all((q['meta_t'] is None or q['pose_t'] <= q['meta_t'] for q in result['samples']))
    assert all((q['meta_t'] != 7.9 for q in result['samples']))
    assert any((q['alignment_clock'] == 'receive_rx' for q in result['samples'])) is False

def test_fitter_distance_bin_cap_limits_near_stationary_tail(tmp_path):
    rows = [_fit_scan(0, 100), _fit_meta(0.1, 100.1, -20)]
    states = {0: {}}
    for i in range(24):
        t = 3 * (i + 1)
        states[t] = dict(x=0.4, y=0.2)
        rows.extend([_fit_scan(t, 100 + t), _fit_meta(t + 0.1, 100 + t + 0.1, -30 - i % 2)])
    result = _fit_tool.fit_recording(_fit_recording(tmp_path, rows), (0, 0), slam=_FitFakeSlam(states), wall_extractor=_fit_extract_known)
    assert result['fit']['sample_count'] == _fit_tool.MAX_SAMPLES_PER_DISTANCE_BIN
    assert result['source']['counts']['distance_bin_cap'] == 16
    assert not result['calibrated']
    assert result['samples'][0]['pose_t'] == 3 and result['samples'][-1]['pose_t'] == 72

@pytest.mark.parametrize('extra', [dict(rx=101, m={'rssi': float('nan')}), dict(rx=101, m={'rssi': '-40'}), {'rx': 101, 'unrecognized': {}}, _fit_scan(2, 99)])
def test_fitter_malformed_recording_is_rejected(tmp_path, extra):
    path = _fit_recording(tmp_path, [_fit_scan(0, 100), extra])
    with pytest.raises(_fit_tool.ReplayError):
        _fit_tool.fit_recording(path, (0, 0), slam=_FitFakeSlam({0: {}, 2: {}}), wall_extractor=_fit_extract_known)

def test_fitter_metadata_only_and_bad_windows_fail(tmp_path):
    path = _fit_recording(tmp_path, [])
    with pytest.raises(_fit_tool.ReplayError, match='no scans'):
        _fit_tool.fit_recording(path, (0, 0), slam=_FitFakeSlam({}), wall_extractor=_fit_extract_known)
    for kwargs in ({'t0': -1}, {'t0': 2, 't1': 1}, {'t1': float('nan')}, {'node_uncertainty_m': -1}):
        with pytest.raises(_fit_tool.ReplayError):
            _fit_tool.fit_recording(path, (0, 0), **kwargs)

def test_fitter_rejected_fit_preserves_existing_output(tmp_path):
    report = _fit_tool.fit_samples([1, 2, 3], [0, 0, 0], [-40, -50, -60])
    output = tmp_path / 'model.json'
    output.write_text('existing reviewed model')
    with pytest.raises(FileExistsError):
        _fit_tool.write_report(output, report)
    assert output.read_text() == 'existing reviewed model'
    output.unlink()
    _fit_tool.write_report(output, report)
    saved = json.loads(output.read_text())
    assert saved['calibrated'] is False
    assert saved['fit']['coefficients'] is not None

def test_fitter_cli_default_output_and_explicit_override(tmp_path, monkeypatch):
    source = tmp_path / 'test.jsonl.gz'
    source.write_bytes(b'fake')
    good = _fit_tool.fit_samples(*_fit_geometry())
    good['source'] = {'counts': {}}
    monkeypatch.setattr(_fit_tool, 'fit_recording', lambda *args, **kwargs: good)
    assert _fit_tool.main([str(source), '--node', '0,0']) == 0
    assert json.loads((tmp_path / 'relay_model.json').read_text())['calibrated']
    out = tmp_path / 'override.json'
    assert _fit_tool.main([str(source), '--node', '0,0', '--output', str(out)]) == 0
    assert out.exists()


@pytest.mark.parametrize('interval,expected', [(2., 2), (1.99999, 1)])
def test_fitter_refresh_spacing_float_boundary(tmp_path, interval, expected):
    event = 2.1 + interval
    states = {0: {}, 2: {}, 4: {}}
    rows = [_fit_scan(0, 0), _fit_meta(.1, .1, -20),
            _fit_scan(2, 2), _fit_meta(2.1, 2.1, -30),
            _fit_scan(4, 4), _fit_meta(event, event, -31)]
    result = _fit_tool.fit_recording(_fit_recording(tmp_path, rows), (1, 0),
                                     slam=_FitFakeSlam(states), wall_extractor=_fit_extract_known)
    assert len(result['samples']) == expected
    if expected == 2:
        assert result['samples'][-1]['meta_t'] == event


@pytest.mark.parametrize('age,expected', [(.5, 1), (.50001, 0)])
def test_fitter_pose_age_float_boundary(tmp_path, age, expected):
    event = 3.9 + age
    rows = [_fit_scan(0, 0), _fit_meta(.1, .1, -20),
            _fit_scan(3.9, 3.9), _fit_meta(event, event, -30)]
    result = _fit_tool.fit_recording(_fit_recording(tmp_path, rows), (1, 0),
                                     slam=_FitFakeSlam({0: {}, 3.9: {}}),
                                     wall_extractor=_fit_extract_known)
    assert len(result['samples']) == expected
    if expected:
        assert result['samples'][0]['pose_age_s'] == event - 3.9
