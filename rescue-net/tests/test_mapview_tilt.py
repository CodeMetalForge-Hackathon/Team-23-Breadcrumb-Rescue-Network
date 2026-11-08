"""A tilted rig: the LiDAR range to a person may be the desk in front of them. People must stay on the map,
one dot each, at their last range measured with the rig level."""
import math

from dashboard.mapview import MapState


def _meta(tilt, r, pid=1, b=0, rs="L"):
    return {"tilt": tilt, "persons": [{"id": pid, "b": b, "r": r, "rs": rs, "ps": "U", "c": 60, "src": "C"}]}


def _ms(tmp_path):
    return MapState(nodes_file=tmp_path / "nodes.json")


def test_tilted_keeps_last_level_range_and_follows_bearing(tmp_path):
    ms = _ms(tmp_path)
    ms.add_meta(_meta(2.0, 3.0), now=100.0)
    p = ms.persons["1"]
    d0 = math.hypot(p["x"], p["y"])
    ms.add_meta(_meta(25.0, 1.2, b=20), now=101.0)     # tilted: 1.2 m is the desk; bearing still good
    assert math.isclose(math.hypot(p["x"], p["y"]), d0, abs_tol=0.02)
    assert p["seen"] == 101.0 and p["b"] == 20 and "rq" not in p
    ms.add_meta(_meta(3.0, 2.5), now=102.0)            # level again: the new range is used
    assert math.isclose(math.hypot(p["x"], p["y"]), 2.5, abs_tol=0.02)


def test_tilted_new_track_same_bearing_is_same_person(tmp_path):
    ms = _ms(tmp_path)
    ms.add_meta(_meta(2.0, 3.0, pid=61, b=352), now=100.0)
    ms.add_meta(_meta(25.0, 1.2, pid=62, b=357), now=102.0)   # tracker re-id while tilted
    ms.add_meta(_meta(25.0, 0.6, pid=68, b=355), now=104.0)
    assert list(ms.persons) == ["68"]                  # one dot, not three
    q = ms.persons["68"]
    assert math.isclose(math.hypot(q["x"], q["y"]), 3.0, abs_tol=0.02)


def test_tilted_first_sighting_still_placed_but_flagged(tmp_path):
    ms = _ms(tmp_path)
    ms.add_meta(_meta(25.0, 1.5, pid=7), now=100.0)
    q = ms.persons["7"]
    assert q["x"] is not None and q["rq"] == "tilt"    # on the map, marked unsure
    ms.add_meta(_meta(2.0, 2.8, pid=7), now=101.0)
    assert "rq" not in q and math.isclose(math.hypot(q["x"], q["y"]), 2.8, abs_tol=0.02)


def test_tilted_box_size_range_used_as_is(tmp_path):
    ms = _ms(tmp_path)
    ms.add_meta(_meta(25.0, 2.0, rs="S"), now=100.0)   # range from the box size does not depend on the LiDAR
    q = ms.persons["1"]
    assert "rq" not in q and math.isclose(math.hypot(q["x"], q["y"]), 2.0, abs_tol=0.02)


def test_two_people_far_apart_in_bearing_stay_two(tmp_path):
    ms = _ms(tmp_path)
    ms.add_meta(_meta(25.0, 2.0, pid=1, b=0), now=100.0)
    ms.add_meta(_meta(25.0, 2.0, pid=2, b=90), now=100.5)
    assert sorted(ms.persons) == ["1", "2"]


def test_fast_reid_level_rig_is_one_dot(tmp_path):
    # live 2026-11-08 07:32: track 86 ended and 89 started within a second at the same spot: two dots
    ms = _ms(tmp_path)
    ms.add_meta(_meta(3.0, 2.0, pid=86, b=340), now=100.0)
    ms.add_meta(_meta(3.0, 2.1, pid=89, b=338), now=100.3)
    assert list(ms.persons) == ["89"]


def test_two_live_tracks_close_together_stay_two(tmp_path):
    ms = _ms(tmp_path)
    both = {"tilt": 3.0, "persons": [{"id": 1, "b": 0, "r": 2.0, "rs": "L"}, {"id": 2, "b": 8, "r": 2.2, "rs": "L"}]}
    ms.add_meta(both, now=100.0)
    ms.add_meta(both, now=100.3)
    assert sorted(ms.persons) == ["1", "2"]
