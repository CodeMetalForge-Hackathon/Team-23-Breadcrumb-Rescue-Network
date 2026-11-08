"""Minimal SLAMTEC RPLIDAR reader (C1 / A-series standard SCAN mode) with a background thread.

latest() returns (angles_deg, dist_mm) of the last complete revolution as numpy arrays, or None.
Protocol: send A5 20, get descriptor A5 5A 05 00 00 40 81, then 5-byte nodes:
  b0 = quality<<2 | !S<<1 | S   (S = start of a new revolution)
  b1 = angle_q6[6:0]<<1 | 1     b2 = angle_q6[14:7]     b3,b4 = distance_q2 (little endian)
"""
import glob
import logging
import threading
import time

import numpy as np

log = logging.getLogger("lidar")

SCAN_DESC = b"\xa5\x5a\x05\x00\x00\x40\x81"


def find_port():
    """The CP210x adapter the C1 ships with; None if not plugged in."""
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if "CP210" in p or "Silicon_Labs" in p:
            return p
    ports = sorted(glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


class Lidar:
    def __init__(self, port=None, baud=460800):
        self.port, self.baud = port or find_port(), baud
        self._lock = threading.Lock()
        self._scan = None
        self.scan_hz = 0.0
        self.status = "starting"
        self._stop = False

    def start(self):
        if not self.port:
            log.warning("lidar: no serial port found yet, will keep looking")
        threading.Thread(target=self._run, name="lidar", daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    def latest(self):
        with self._lock:
            return self._scan

    def _run(self):
        import os
        import serial
        while not self._stop:
            if not self.port or not os.path.exists(self.port):     # plugged in late, or re-enumerated
                self.port = find_port()
                if not self.port:
                    self.status = "no lidar port"
                    time.sleep(2)
                    continue
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    ser.dtr = False                       # A-series adapters: DTR low = motor on
                    self._read_scans(ser)
            except Exception as e:                        # unplugged, garbage, ... retry
                self.status = f"error: {e}"
                log.warning("lidar: %s, retrying", e)
            time.sleep(2)

    def _read_scans(self, ser):
        ser.write(b"\xa5\x25")                            # STOP any previous scan
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(b"\xa5\x20")                            # SCAN
        buf = ser.read(7)
        if buf != SCAN_DESC:
            raise IOError(f"bad scan descriptor {buf.hex()}")
        self.status = "ok"
        angles, dists = [], []
        t_last = time.time()
        pending = b""
        last_data = time.time()
        while not self._stop:
            chunk = ser.read(500)
            if chunk:
                last_data = time.time()
            elif time.time() - last_data > 6:            # motor spin-up takes a moment
                raise IOError("no data")
            data = pending + chunk
            n = len(data) // 5
            pending = data[n * 5:]
            i = 0
            while i < n:
                b = data[i * 5:i * 5 + 5]
                s, ns, c = b[0] & 1, (b[0] >> 1) & 1, b[1] & 1
                if s == ns or c != 1:                     # out of sync: shift by one byte
                    rest = data[i * 5 + 1:]
                    k = len(rest) // 5
                    data, n, i = rest, k, 0
                    pending = rest[k * 5:]
                    continue
                if s and len(angles) > 50:
                    now = time.time()
                    self.scan_hz = 0.8 * self.scan_hz + 0.2 / max(now - t_last, 1e-3)
                    t_last = now
                    with self._lock:
                        self._scan = (np.array(angles, np.float32), np.array(dists, np.float32))
                    angles, dists = [], []
                dist = (b[3] | (b[4] << 8)) / 4.0
                if dist > 0:
                    angles.append(((b[1] >> 1) | (b[2] << 7)) / 64.0)
                    dists.append(dist)
                i += 1


def render(cv2, scan, size=480, max_m=6.0, status="", hz=0.0, persons=(), fovs=()):
    """Top-down plan view: lidar at the centre, 0 deg pointing up, 1 m rings.
    persons: dicts with b (deg), r (m or None), id, ps; fovs: (yaw, hfov, BGR colour) wedges of the cameras."""
    img = np.full((size, size, 3), 18, np.uint8)
    c = size // 2
    px_per_m = (size / 2 - 10) / max_m
    for r in range(1, int(max_m) + 1):
        cv2.circle(img, (c, c), int(r * px_per_m), (55, 55, 55), 1)
        cv2.putText(img, f"{r}m", (c + 3, c - int(r * px_per_m) + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1)
    cv2.line(img, (c, 0), (c, size), (40, 40, 40), 1)
    cv2.line(img, (0, c), (size, c), (40, 40, 40), 1)
    n = 0
    if scan is not None:
        a, d = scan
        th = np.radians(a)
        x = (c + np.sin(th) * d / 1000.0 * px_per_m).astype(int)
        y = (c - np.cos(th) * d / 1000.0 * px_per_m).astype(int)
        ok = (x >= 0) & (x < size) & (y >= 0) & (y < size)
        img[y[ok], x[ok]] = (80, 255, 80)
        for dx, dy in ((1, 0), (0, 1), (1, 1)):          # thicker dots
            xx, yy = np.clip(x[ok] + dx, 0, size - 1), np.clip(y[ok] + dy, 0, size - 1)
            img[yy, xx] = (80, 255, 80)
        n = int(ok.sum())
    for yaw, hfov, color in fovs:                       # camera fields of view
        for a in (yaw - hfov / 2, yaw + hfov / 2):
            th = np.radians(a)
            cv2.line(img, (c, c), (int(c + np.sin(th) * size), int(c - np.cos(th) * size)), color, 1)
    for p in persons:                                    # tracked people
        th, color = np.radians(p["b"]), (0, 0, 255) if p.get("ps") == "D" else (0, 220, 255)
        if p.get("r") is None:                           # bearing only: a ray
            cv2.line(img, (c, c), (int(c + np.sin(th) * size), int(c - np.cos(th) * size)), color, 2)
            org = (int(c + np.sin(th) * 0.8 * c), int(c - np.cos(th) * 0.8 * c))
        else:
            org = (int(c + np.sin(th) * p["r"] * px_per_m), int(c - np.cos(th) * p["r"] * px_per_m))
            cv2.circle(img, org, 9, color, 2)
        label = f"#{p.get('id', '?')} {p['b']:.0f}deg" + ("" if p.get("r") is None else f" {p['r']:.1f}m")
        cv2.putText(img, label, (org[0] + 11, org[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    cv2.circle(img, (c, c), 5, (0, 160, 255), -1)
    cv2.line(img, (c, c), (c, c - 18), (0, 160, 255), 2)
    cv2.putText(img, f"LIDAR {status}  {hz:.1f} Hz  {n} pts  range {max_m:.0f} m",
                (6, size - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
    return img


if __name__ == "__main__":                                # quick check: python3 -m spotter.lidar
    import sys
    logging.basicConfig(level=logging.INFO)
    lid = Lidar(sys.argv[1] if len(sys.argv) > 1 else None).start()
    for _ in range(10):
        time.sleep(1)
        s = lid.latest()
        print(lid.port, lid.status, f"{lid.scan_hz:.1f}Hz",
              "none" if s is None else f"{len(s[0])} pts, min {s[1].min():.0f} mm, max {s[1].max():.0f} mm")
    lid.stop()
