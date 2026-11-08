"""Offline SLAM + Cloud replay; optional private recordings stay untracked.

From rescue-net:
  python -m tests.test_cloud_filter_replay mapping/data/walk2.jsonl.gz \
      --output /tmp/walk2-cloud.json

The exported scene contains both the complete cloud and the default visibility
mask, so the renderer can compare the same accepted poses with its toggle.
"""

import argparse
import gzip
import json
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np

from dashboard.cloud import Cloud, LEVEL_REFERENCE
from mapping.replay import _scan
from mapping.slam import Slam2D


def replay_cloud(input_path):
    slam, cloud = Slam2D(), Cloud()
    route, reasons, scans = [], Counter(), 0
    started = time.perf_counter()
    with gzip.open(input_path, "rt", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            if "ang" not in row or "mm" not in row:
                continue
            # tap_walk stores angles in centidegrees and gravity under "a".
            angles, distances, stamp, acc = _scan(row, line_number)
            result = slam.update(angles, distances, t=stamp, acc=acc)
            pose, ok = slam.pose, result["ok"]
            cloud.add_scan(pose, angles, distances, acc, ok)
            scans += 1
            if not ok:
                reasons[result["reason"]] += 1
            elif not route or math.hypot(pose[0] - route[-1][0], pose[1] - route[-1][1]) > .05:
                route.append(list(pose[:2]))
                if len(route) > 5000:
                    route = route[::2]
    scene = cloud.snapshot(pose=slam.pose, path=route)
    report = {"scans": scans, "accepted": scene["accepted"], "rejections": dict(reasons),
              "points": scene["count"], "visible": scene["visible_count"], "hidden": scene["hidden_count"],
              "surface_filter": scene["surface_filter"], "pose": scene["pose"],
              "elapsed_s": round(time.perf_counter() - started, 3),
              "payload_bytes": len(json.dumps(scene).encode("utf-8"))}
    return scene, report


def test_replay_uses_tap_walk_centidegrees_and_acceleration(tmp_path):
    path = tmp_path / "walk.jsonl.gz"
    with gzip.open(path, "wt") as out:
        out.write(json.dumps({"meta": {"source": "synthetic"}}) + "\n")
        for i in range(3):
            out.write(json.dumps({"t": 10 + i * .1, "ang": (np.arange(360) * 100).tolist(),
                                  "mm": [2000] * 360, "a": LEVEL_REFERENCE.tolist()}) + "\n")
    scene, report = replay_cloud(path)
    assert report["scans"] == report["accepted"] == 3
    assert 200 < scene["count"] <= 360
    assert scene["visible_count"] == scene["count"] and scene["hidden_count"] == 0
    assert scene["tilt"] is True


def test_replay_never_integrates_rejected_imu_scan(tmp_path):
    path = tmp_path / "tilted.jsonl.gz"
    with gzip.open(path, "wt") as out:
        out.write(json.dumps({"t": 10, "ang": (np.arange(360) * 100).tolist(),
                              "mm": [2000] * 360, "a": [21, 0, 0]}) + "\n")
    scene, report = replay_cloud(path)
    assert report["scans"] == 1 and report["accepted"] == 0
    assert report["rejections"] == {"tilted": 1}
    assert scene["count"] == scene["visible_count"] == scene["hidden_count"] == 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.recording.resolve() == args.output.resolve():
        parser.error("output must not overwrite the recording")
    scene, report = replay_cloud(args.recording)
    args.output.write_text(json.dumps(scene, separators=(",", ":")), encoding="utf-8")
    print(json.dumps(report, indent=2))
