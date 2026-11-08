"""A tilted rig must not move people on the map with LiDAR ranges (the beam hits the desk in front of them)."""
from dashboard.mapview import MapState


def _meta(tilt, r, pid=1, rs="L"):
    return {"tilt": tilt, "persons": [{"id": pid, "b": 0, "r": r, "rs": rs, "ps": "U", "c": 60, "src": "C"}]}


def test_tilted_lidar_range_keeps_last_position(tmp_path):
    ms = MapState(nodes_file=tmp_path / "nodes.json")
    ms.add_meta(_meta(2.0, 3.0), now=100.0)
    p = ms.persons["1"]
    x0, y0 = p["x"], p["y"]
    ms.add_meta(_meta(25.0, 1.2), now=101.0)           # tilted: the 1.2 m return is the desk, not the person
    assert (p["x"], p["y"]) == (x0, y0)
    assert p["seen"] == 101.0                          # still counted as seen
    ms.add_meta(_meta(3.0, 2.5), now=102.0)            # level again: the range is used
    assert (p["x"], p["y"]) != (x0, y0)


def test_tilted_new_track_gets_no_position(tmp_path):
    ms = MapState(nodes_file=tmp_path / "nodes.json")
    ms.add_meta(_meta(25.0, 0.6, pid=7), now=100.0)
    assert ms.persons["7"]["x"] is None                # listed, but no dot at a wrong spot


def test_tilted_box_size_range_still_used(tmp_path):
    ms = MapState(nodes_file=tmp_path / "nodes.json")
    ms.add_meta(_meta(25.0, 2.0, rs="S"), now=100.0)   # range from the box size does not depend on the LiDAR
    assert ms.persons["1"]["x"] is not None
