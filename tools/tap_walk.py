#!/usr/bin/env python3
"""Record a walk from the live bulk link (what the dashboard sees), without stopping the spotter or touching the Pi.

Tails BASE's serial lines through the board hub, rebuilds the LiDAR scans (bulk kind 3) and writes them in the
tools/record_walk.py format, plus the spotter meta and the node messages (MARK, PERSON, ...):
    python3 tools/tap_walk.py rescue-net/mapping/data/walk2.jsonl.gz 240

Output (gzip JSON lines): {"meta": {...}} first, then
    {"t": spotter unix s, "rx": laptop rx s, "a": [ax, ay, az] or null, "ang": [deg*100], "mm": [mm]}   one per scan
    {"rx": s, "m": {...}}     spotter meta (people with bearing/range, kp, lvl, rssi, ...)
    {"rx": s, "msg": {...}}   relay-chain message (MARK = node button D2 pressed while the spotter stood next to it)
Bins are whole degrees (nearest return per degree, as on the bulk link).
"""
import gzip
import json
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "rescue-net"))
from dashboard.remote import Remote  # noqa: E402

HUB = os.environ.get("HUB", "http://100.115.103.120:8765")
TOKEN = open(os.path.expanduser("~/.config/board_hub_token")).read().strip()


def poll(since, wait=5):
    req = urllib.request.Request(f"{HUB}/log/BASE?since={since}&wait={wait}", headers={"X-Token": TOKEN})
    with urllib.request.urlopen(req, timeout=wait + 15) as r:
        return json.load(r)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "walk.jsonl.gz"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 240
    f = gzip.open(out, "wt")
    n = {"scan": 0, "meta": 0, "msg": 0}

    def on_object(name, blob, parsed):
        now = round(time.time(), 3)
        if name == "lidar" and parsed:
            mm, t, acc = parsed
            idx = [i for i, v in enumerate(mm) if v > 0]
            f.write(json.dumps({"t": t, "rx": now, "a": list(acc) if acc else None,
                                "ang": [i * 100 for i in idx], "mm": [mm[i] for i in idx]}) + "\n")
            n["scan"] += 1
        elif name == "meta" and parsed:
            f.write(json.dumps({"rx": now, "m": {k: v for k, v in parsed.items() if k != "_rx"}}) + "\n")
            n["meta"] += 1

    rem = Remote(on_object=on_object)
    f.write(json.dumps({"meta": {"source": "bulk link via BASE (tools/tap_walk.py)", "bins": "1 deg, nearest",
                                 "acc_ref_level": [1.45, 1.33, 21.26], "acc_counts_per_g": 21.33,
                                 "lidar_mount_tilt_deg": 6,
                                 "angle": "deg clockwise from LiDAR front, point = (sin(a)*d, cos(a)*d)"}}) + "\n")
    req = urllib.request.Request(f"{HUB}/log/BASE?n=1", headers={"X-Token": TOKEN})
    with urllib.request.urlopen(req, timeout=15) as r:
        since = max((l[0] for l in json.load(r).get("lines", [])), default=0)
    t0, last_print, gaps = time.time(), 0.0, 0
    try:
        while time.time() - t0 < secs:
            try:
                d = poll(since)
            except OSError as e:
                print("poll:", e, flush=True)
                time.sleep(1)
                continue
            lines = d.get("lines", [])
            if lines and lines[0][0] > since + 1:
                gaps += 1                                   # hub buffer (3000 lines) overran between polls
            for seq, _ts, text in lines:
                since = max(since, seq)
                if text.startswith("$B"):
                    rem.add(text)
                elif text.startswith("{"):
                    try:
                        msg = json.loads(text)
                    except ValueError:
                        continue
                    if msg.get("t") not in (None, "HB"):
                        f.write(json.dumps({"rx": round(time.time(), 3), "msg": msg}) + "\n")
                        n["msg"] += 1
                        print("msg:", text[:120], flush=True)
            if time.time() - last_print > 10:
                last_print = time.time()
                print(f"{time.time() - t0:5.0f} s  {n}  gaps {gaps}", flush=True)
    finally:
        f.close()
    print(f"done: {n} gaps {gaps} -> {out}")


if __name__ == "__main__":
    main()
