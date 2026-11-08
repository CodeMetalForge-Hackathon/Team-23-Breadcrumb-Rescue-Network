import base64
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from dashboard.cloud import Cloud, LEVEL_REFERENCE


def decoded(snapshot):
    integers = np.frombuffer(base64.b64decode(snapshot["points"]), dtype="<i2").reshape(-1, 3)
    assert len(integers) == snapshot["count"]
    return np.asarray(snapshot["origin"]) + integers * snapshot["scale"]


def test_map_heading_clockwise_from_y_and_mm_to_metres():
    cloud = Cloud()
    cloud.add_scan((2, 3, math.pi / 2), [0, 90, 180, 270], [1000] * 4, None, True)
    np.testing.assert_allclose(decoded(cloud.snapshot()), [[3, 3, 0], [2, 2, 0], [1, 3, 0], [2, 4, 0]], atol=0.006)


def test_level_reference_and_missing_acceleration_are_flat():
    for acc in (None, LEVEL_REFERENCE, [0, 0, 0], [float("nan"), 0, 21], [0, 0, 80], [1, 2]):
        cloud = Cloud()
        cloud.add_scan((0, 0, 0), [0, 90, 180, 270], [2000] * 4, acc, True)
        np.testing.assert_allclose(decoded(cloud.snapshot())[:, 2], 0, atol=0.006)
        assert cloud.snapshot()["tilt"] == (acc is LEVEL_REFERENCE)


def test_known_tilt_sign_and_projection_length():
    angle = math.radians(5)
    rotation = np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0],
                         [-math.sin(angle), 0, math.cos(angle)]])
    acc = rotation @ LEVEL_REFERENCE
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [90, 270], [4000, 4000], acc, True)
    points = decoded(cloud.snapshot())
    assert points[0, 2] > 0.3 and points[1, 2] < -0.3
    np.testing.assert_allclose(np.linalg.norm(points, axis=1), 4, atol=0.012)
    assert cloud.snapshot()["tilt"] is True


def test_rejected_scan_does_not_pollute_map():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [0], [2000], None, True)
    before = cloud.snapshot()
    cloud.add_scan((10, 20, 1), [0, 90], [20000, 20000], LEVEL_REFERENCE, False)
    after = cloud.snapshot()
    assert after["points"] == before["points"] and after["v"] == before["v"]
    assert after["scan_ok"] is False and after["rejected"] == 1 and after["accepted"] == 1
    assert after["pose"] == [0, 0, 0]


def test_voxel_dedup_and_recent_points_survive_capacity():
    cloud = Cloud(max_points=3)
    cloud.add_scan((0, 0, 0), [0, 0, 0, 90], [1010, 1011, 1012, 1000], None, True)
    assert cloud.snapshot()["count"] == 2
    cloud.add_scan((0, 0, 0), [180, 270], [2000, 2000], None, True)
    points = decoded(cloud.snapshot())
    assert len(points) == 3
    assert any(np.allclose(point, [0, -2, 0], atol=0.01) for point in points)
    assert not any(np.allclose(point, [0, 1.01, 0], atol=0.02) for point in points)


def test_invalid_ranges_and_pose_are_ignored():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [0, 30, 60, 90, 120, float("nan")],
                   [0, -1, 200, float("inf"), 31000, 1000], None, True)
    assert cloud.snapshot()["count"] == 0
    for pose, angles, distances in [((0, float("nan"), 0), [0], [1000]),
                                    ((0, 0), [0], [1000]), ((0, 0, 0), [0, 1], [1000]),
                                    ((1e12, 0, 0), [0], [1000])]:
        cloud.add_scan(pose, angles, distances, None, True)
    assert cloud.snapshot()["count"] == 0


def test_maximum_cloud_and_scene_stay_below_400_kb():
    cloud = Cloud()
    angles = np.arange(360)
    for index in range(120):
        cloud.add_scan((index * 0.13, index * 0.07, index * 0.01), angles,
                       5000 + (angles * 109 + index * 173) % 15000,
                       LEVEL_REFERENCE + [math.sin(index) * 1.5, math.cos(index) * 1.5, 0], True)
    people = [{"id": str(i), "x": i / 3, "y": 2, "ps": "D", "kp": [99] * 34} for i in range(128)]
    snap = cloud.snapshot(persons=people, path=[[i / 7, i / 13] for i in range(5000)],
                          nodes={f"N{i}": {"x": i, "y": 3} for i in range(32)})
    assert snap["count"] == 30000
    assert len(snap["path"]) <= 800
    assert len(json.dumps(snap, allow_nan=False).encode()) < 400000
    assert decoded(snap).shape == (30000, 3)


def test_scene_filters_stale_people_preserves_keypoints_and_handles_unknown_location():
    cloud = Cloud()
    keypoints = [10, 20] * 17
    snap = cloud.snapshot(persons=[{"id": 1, "x": None, "y": None, "ps": "D", "kp": keypoints, "age": 2},
                                   {"id": 2, "x": 2, "y": 3, "seen": time.time() - 200},
                                   {"id": 3, "x": float("nan"), "y": 1, "kp": [float("nan")] * 34}],
                          pose=np.array([1.0, 2.0, 0.3]), path=[[0, 0], [float("nan"), 0], [2, 3]],
                          nodes={"B2": {"x": 1, "y": 2}, "bad": {"x": None, "y": 1}})
    assert [p["id"] for p in snap["persons"]] == ["1", "3"]
    assert snap["persons"][0]["kp"] == keypoints
    assert snap["persons"][1]["kp"] == [-1] * 34
    assert snap["persons"][1]["x"] is None
    assert snap["path"] == [[0, 0], [2, 3]]
    assert snap["nodes"] == {"B2": {"x": 1, "y": 2}}
    json.dumps(snap, allow_nan=False)
    keypoints[0] = 90
    assert snap["persons"][0]["kp"][0] == 10


def test_large_map_quantization_does_not_wrap():
    cloud = Cloud()
    cloud.add_scan((-800, 200, 0), [0], [1000], None, True)
    cloud.add_scan((800, 200, 0), [0], [1000], None, True)
    snap = cloud.snapshot()
    np.testing.assert_allclose(decoded(snap), [[-800, 201, 0], [800, 201, 0]], atol=snap["scale"])


def test_clear_invalidates_version_and_resets_counts():
    cloud = Cloud()
    cloud.add_scan((0, 0, 0), [0], [1000], LEVEL_REFERENCE, True)
    before = cloud.snapshot()
    cloud.clear()
    after = cloud.snapshot()
    assert after["v"] > before["v"] and after["count"] == 0 and after["points"] == ""
    assert after["accepted"] == 0 and after["scan_age"] is None


def test_add_and_poll_concurrently_have_consistent_binary_snapshots():
    cloud = Cloud(max_points=600)

    def write():
        for i in range(50):
            cloud.add_scan((i / 20, 0, 0), np.arange(360), np.full(360, 2000), LEVEL_REFERENCE, True)

    def read():
        for _ in range(30):
            snap = cloud.snapshot()
            assert len(decoded(snap)) == snap["count"] <= 600
            json.dumps(snap, allow_nan=False)

    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(write), pool.submit(read)]
        for future in futures:
            future.result()


@pytest.mark.parametrize("kwargs", [{"voxel_m": 0}, {"voxel_m": float("nan")},
                                    {"max_points": 0}, {"max_points": 30001}])
def test_invalid_resource_limits_fail_early(kwargs):
    with pytest.raises(ValueError):
        Cloud(**kwargs)
