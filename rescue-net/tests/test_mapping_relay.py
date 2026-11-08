import math

import numpy as np
import pytest

from mapping.relay import MODEL, count_wall_segments, suggest_relays


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
