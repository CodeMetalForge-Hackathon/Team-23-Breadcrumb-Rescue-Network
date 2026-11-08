"""Synthetic room and Flask integration; no board, Pi, camera or hub access.

Manual preview, from rescue-net:
    python -m tests.test_cloud_scene --port 5072
"""

import argparse
import json
import math
import time

import numpy as np

from dashboard.app import State, create_app
from dashboard.cloud import LEVEL_REFERENCE


def room_scan(pose):
    x, y, heading = pose
    angles = np.deg2rad(np.arange(360)) + heading
    dx, dy = np.sin(angles), np.cos(angles)
    with np.errstate(divide="ignore", invalid="ignore"):
        rx = np.where(dx > 0, (4 - x) / dx, (-4 - x) / dx)
        ry = np.where(dy > 0, (5 - y) / dy, (-3 - y) / dy)
    rx = np.where(np.abs(dx) < 1e-10, np.inf, rx)
    ry = np.where(np.abs(dy) < 1e-10, np.inf, ry)
    return np.minimum(rx, ry) * 1000


def synthetic_state():
    state = State()
    state.map.slam = None  # exercise Cloud with known poses, not a second SLAM implementation
    for i in range(90):
        theta = i * math.pi / 60
        pose = (1.2 * math.sin(theta), 0.7 * math.cos(theta) - 0.7, 0.1 * math.sin(theta))
        state.map.pose = pose
        acc = LEVEL_REFERENCE + np.array([1.6 * math.sin(theta), 1.6 * math.cos(theta), 0])
        state.map.add_scan(room_scan(pose), acc=acc)
    state.map.nodes = {"BASE": {"x": 0, "y": -2.5}, "B1": {"x": -2.0, "y": -0.8},
                       "B2": {"x": 0.5, "y": 0.8}}
    kp = [50, 5, 45, 4, 55, 4, 37, 8, 63, 8, 30, 23, 70, 23, 20, 42, 80, 42,
          12, 58, 88, 58, 38, 55, 62, 55, 35, 77, 65, 77, 32, 99, 68, 99]
    now = time.time()
    state.map.persons = {"1": {"id": "1", "x": -1.2, "y": 2.0, "ps": "U", "kp": kp, "seen": now, "src": "CT"},
                         "2": {"id": "2", "x": 1.7, "y": 2.1, "ps": "D", "kp": kp, "seen": now, "src": "C"},
                         "3": {"id": "3", "x": 2.5, "y": -0.5, "ps": "U", "seen": now, "src": "C"},
                         "4": {"id": "4", "x": None, "y": None, "ps": None, "seen": now, "src": "T"}}
    return state


def test_cloud_endpoint_and_offline_assets():
    state = synthetic_state()
    client = create_app(state).test_client()
    response = client.get("/api/cloud")
    assert response.status_code == 200
    data = response.get_json()
    assert 1000 < data["count"] <= 30000
    assert len(response.data) < 400000
    assert data["bounds"][0][2] < -0.1 and data["bounds"][1][2] > 0.1
    assert len(data["path"]) > 10 and "B2" in data["nodes"]
    assert any(person.get("kp") for person in data["persons"])
    json.dumps(data, allow_nan=False)
    for url in ["/", "/static/vendor/three.min.js", "/static/view3d.js"]:
        asset = client.get(url)
        assert asset.status_code == 200
    assert b"/static/vendor/three.min.js" in client.get("/").data
    assert b"/static/view3d.js" in client.get("/").data


if __name__ == "__main__":
    import threading

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5072)
    args = parser.parse_args()
    state = synthetic_state()

    def feed():
        started = time.monotonic()
        while True:
            theta = (time.monotonic() - started) * 0.3
            pose = (1.2 * math.sin(theta), 0.7 * math.cos(theta) - 0.7, 0.1 * math.sin(theta))
            state.map.pose = pose
            acc = LEVEL_REFERENCE + np.array([1.6 * math.sin(theta), 1.6 * math.cos(theta), 0])
            state.map.add_scan(room_scan(pose), acc=acc)
            with state.map.lock:
                for person in state.map.persons.values():
                    person["seen"] = time.time()
            time.sleep(0.1)

    threading.Thread(target=feed, daemon=True).start()
    create_app(state).run(host="127.0.0.1", port=args.port, use_reloader=False)
