#!/usr/bin/env python3
"""Record a handheld walk for SLAM development: every LiDAR revolution plus the accelerometer reading.

Run on the Pi from the spotter checkout while rescue-spotter is stopped (the LiDAR port is exclusive):
    cd ~/rescue-net-dev && python3 /tmp/record_walk.py ~/logs/walk1.jsonl.gz 150

Output (gzip JSON lines): first {"meta": {...}}, then one line per revolution
    {"t": unix s, "a": [ax, ay, az] MMA7660 counts (~21.33 per g) or null, "ang": [deg*100], "mm": [dist mm]}
"""
import gzip
import json
import sys
import time

sys.path.insert(0, ".")
from spotter.lidar import Lidar  # noqa: E402

ACC = 0x4C


def acc_open():
    try:
        from smbus2 import SMBus
        bus = SMBus(1)
        bus.write_byte_data(ACC, 0x07, 0x00)
        bus.write_byte_data(ACC, 0x08, 0x00)
        bus.write_byte_data(ACC, 0x07, 0x01)
        return bus
    except Exception as e:
        print("no accelerometer:", e)
        return None


def acc_read(bus):
    if bus is None:
        return None
    for _ in range(3):
        try:
            r = bus.read_i2c_block_data(ACC, 0x00, 3)
        except OSError:
            return None
        if not any(v & 0x40 for v in r):
            return [(v & 0x3F) - 64 if v & 0x20 else v & 0x3F for v in r]
    return None


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "walk.jsonl.gz"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 120
    bus = acc_open()
    lid = Lidar().start()
    t_end = time.time() + 6
    while lid.latest() is None and time.time() < t_end:
        time.sleep(0.1)
    if lid.latest() is None:
        sys.exit(f"no LiDAR data ({lid.port}, {lid.status})")
    meta = {"rig": "handheld Pi5 + RPLIDAR C1 + MMA7660", "lidar_port": lid.port,
            "acc_ref_level": [1.45, 1.33, 21.26], "acc_counts_per_g": 21.33,
            "lidar_mount_tilt_deg": 6, "self_hit": "120-240 deg at 10-14 cm",
            "angle": "deg clockwise from LiDAR front, point = (sin(a)*d, cos(a)*d)"}
    n, last, t0 = 0, None, time.time()
    with gzip.open(out, "wt") as f:
        f.write(json.dumps({"meta": meta}) + "\n")
        while time.time() - t0 < secs:
            s = lid.latest()
            if s is not None and s is not last:
                last = s
                a, d = s
                f.write(json.dumps({"t": round(time.time(), 3), "a": acc_read(bus),
                                    "ang": [int(round(x * 100)) for x in a], "mm": [int(x) for x in d]}) + "\n")
                n += 1
                if n % 50 == 0:
                    print(f"{time.time() - t0:5.0f} s  {n} scans  {lid.scan_hz:.1f} Hz", flush=True)
            time.sleep(0.01)
    lid.stop()
    print(f"done: {n} scans in {time.time() - t0:.0f} s -> {out}")


if __name__ == "__main__":
    main()
