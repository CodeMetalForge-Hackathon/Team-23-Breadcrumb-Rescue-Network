import numpy as np
import pytest

from spotter.config import DEFAULTS
from spotter.detector import Detection
from spotter.fusion import (analyze, box_to_thermal, cam_to_cell, cam_to_thermal, classify_person,
                            find_heat_blobs, inside_fraction, thermal_to_cam, view_of)


def cfg(**over):
    return {**DEFAULTS, **over}


def room(temp=22.0):
    return np.full((24, 32), temp)


def person(x0, y0, x1, y1, score=0.9):
    return Detection("person", score, x0, y0, x1, y1)


# ---- mapping ----

def test_identity_mapping_centre():
    assert cam_to_cell(0.5, 0.5, cfg()) == (16.0, 12.0)


def test_mapping_clamps_to_grid():
    col, row = cam_to_cell(1.5, -0.5, cfg())
    assert 31 < col < 32 and row == 0.0


def test_flip_h_mirrors():
    assert cam_to_thermal(0.2, 0.5, cfg(flip_h=True)) == pytest.approx((0.8, 0.5))


def test_scale_and_offset():
    tu, tv = cam_to_thermal(1.0, 0.0, cfg(scale_x=0.5, scale_y=2.0, offset_x=0.1, offset_y=0.0))
    assert (tu, tv) == pytest.approx((0.85, -0.5))


@pytest.mark.parametrize("c", [cfg(), cfg(flip_h=True, flip_v=True), cfg(scale_x=0.8, offset_x=-0.1, offset_y=0.05)])
def test_thermal_to_cam_inverts(c):
    assert thermal_to_cam(*cam_to_thermal(0.3, 0.7, c), c) == pytest.approx((0.3, 0.7))


def test_box_sorted_when_flipped():
    tu0, tv0, tu1, tv1 = box_to_thermal(person(0.1, 0.2, 0.3, 0.4), cfg(flip_h=True))
    assert tu0 < tu1 and tu0 == pytest.approx(0.7)


def test_inside_fraction():
    assert inside_fraction((0.2, 0.2, 0.4, 0.4)) == pytest.approx(1.0)
    assert inside_fraction((0.8, 0.0, 1.2, 1.0)) == pytest.approx(0.5)
    assert inside_fraction((1.1, 0.0, 1.3, 1.0)) == 0.0


def test_view_thirds():
    assert [view_of(u) for u in (0.0, 0.32, 0.34, 0.66, 0.67, 1.0)] == ["L", "L", "C", "C", "R", "R"]


# ---- person decisions ----

def test_hot_person_is_confirmed():
    t = room()
    t[8:16, 14:18] = 31.0
    decision, dt = classify_person(person(0.4, 0.3, 0.6, 0.7), t, cfg())
    assert decision == "confirmed" and dt == pytest.approx(9.0)


def test_lukewarm_person_is_uncertain():
    t = room()
    t[8:16, 14:18] = 25.5
    assert classify_person(person(0.4, 0.3, 0.6, 0.7), t, cfg())[0] == "uncertain"


def test_poster_is_rejected():
    t = room()
    t[8:16, 2:6] = 31.0                       # someone warm elsewhere; the poster box itself is room temperature
    assert classify_person(person(0.4, 0.3, 0.6, 0.7), t, cfg())[0] == "rejected"


def test_no_thermal_means_uncertain():
    assert classify_person(person(0.4, 0.3, 0.6, 0.7), None, cfg()) == ("uncertain", None)


def test_box_mostly_outside_thermal_view_is_uncertain():
    t = room()
    c = cfg(scale_x=0.5, offset_x=0.7)       # camera right half maps beyond the thermal frame
    assert classify_person(person(0.7, 0.3, 0.95, 0.7), t, c) == ("uncertain", None)


def test_tiny_box_uses_at_least_one_pixel():
    t = room()
    t[12, 16] = 30.0
    assert classify_person(person(0.51, 0.51, 0.511, 0.511), t, cfg())[0] == "confirmed"


# ---- HEAT blobs ----

def test_heat_blob_found_with_view():
    t = room()
    t[18:21, 27:30] = 30.0
    [b] = find_heat_blobs(t, cfg())
    assert b.pixels == 9 and b.dt == pytest.approx(8.0) and view_of(b.u) == "R"


def test_heat_blob_inside_person_box_ignored():
    t = room()
    t[18:21, 27:30] = 30.0
    assert find_heat_blobs(t, cfg(), [box_to_thermal(person(0.8, 0.7, 0.95, 0.9), cfg())]) == []


def test_small_blob_ignored():
    t = room()
    t[5, 5] = t[5, 6] = 30.0
    assert find_heat_blobs(t, cfg()) == []


def test_diagonal_pixels_are_separate_blobs():
    t = room()
    t[5, 5] = t[6, 6] = t[7, 7] = 30.0
    assert find_heat_blobs(t, cfg()) == []


def test_analyze_candidates():
    t = room()
    t[8:16, 14:18] = 31.0                     # person in the centre
    t[18:21, 27:30] = 30.0                    # radiator bottom-right
    a = analyze([person(0.4, 0.3, 0.6, 0.7, 0.876), person(0.05, 0.1, 0.2, 0.5)], t, cfg())
    assert {"t": "PERSON", "c": 88, "th": 1, "v": "C", "dt": 9} in a.candidates
    assert {"t": "HEAT", "c": 40, "v": "R", "dt": 8} in a.candidates
    assert [p.decision for p in a.persons] == ["confirmed", "rejected"]
    assert len(a.candidates) == 2


def test_recent_boxes_suppress_heat():
    t = room()
    t[8:16, 14:18] = 31.0                     # body heat, but the detector missed this frame
    assert analyze([], t, cfg(), [person(0.4, 0.3, 0.6, 0.7)]).candidates == []
    assert analyze([], t, cfg()).candidates[0]["t"] == "HEAT"


# ---- other sensor sizes (e.g. a 160x120 network thermal camera) ----

def test_any_grid_size():
    t = np.full((120, 160), 22.0)
    t[40:80, 70:90] = 31.0                    # person in the centre
    t[95:105, 140:150] = 30.0                 # radiator bottom-right
    a = analyze([person(0.4, 0.3, 0.6, 0.7)], t, cfg(heat_min_pixels=20))
    assert {"t": "PERSON", "c": 90, "th": 1, "v": "C", "dt": 9} in a.candidates
    assert {"t": "HEAT", "c": 40, "v": "R", "dt": 8} in a.candidates
    assert cam_to_cell(0.5, 0.5, cfg(), (120, 160)) == (80.0, 60.0)
