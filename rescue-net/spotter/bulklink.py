"""Bulk uplink over our own relay chain (no venue Wi-Fi, no internet):

  Pi --Wi-Fi (B2's hidden RESCUE-LINK AP, UDP 192.168.4.1:5005)--> B2 --ESP-NOW--> B1 --> BASE --> dashboard

Objects, each split into chunks  B7 01 <hop=3> <kind> <seq u16> <idx u16> <cnt u16> + payload (<= 230 B), little endian:
  1 camera JPEG, 2 thermal JPEG,
  3 LiDAR: 360 x uint16 mm per whole degree (0 = no return), then t (float64 unix s) and the accelerometer
    ax, ay, az (int8 counts, ~21 per g, 0,0,0 if none)  -> 731 bytes,
  4 meta: UTF-8 JSON {"t", "lvl", "rssi", ...meta_fn()}  (people with bearing/range, tilt, ...).
The map needs LiDAR + meta most, so they go first and fastest; the pictures get whatever is left.
Link level (auto from the Pi's Wi-Fi signal to B2, or forced with level=0/1/2):
  0 good: LiDAR 10 Hz, meta 4 Hz, each picture 2 fps at 320 px
  1 fair: LiDAR 10 Hz, meta 2 Hz, each picture 1 fps at 240 px
  2 poor: LiDAR 5 Hz,  meta 2 Hz, each picture every 4 s at 160 px (map and people keep flowing)
Alerts do not use this: they keep going over USB -> SP1 -> LoRa.
"""
import json
import logging
import socket
import struct
import threading
import time

import numpy as np

log = logging.getLogger("bulklink")

KIND_CAMERA, KIND_THERMAL, KIND_LIDAR, KIND_META = 1, 2, 3, 4
PAYLOAD = 230
LEVELS = [
    dict(lidar=0.1, meta=0.25, image=0.5, cam=(320, 240), th=(320, 256), q=45),
    dict(lidar=0.1, meta=0.5, image=1.0, cam=(240, 180), th=(240, 192), q=40),
    dict(lidar=0.2, meta=0.5, image=4.0, cam=(160, 120), th=(160, 128), q=35),
]
RSSI_FAIR, RSSI_POOR, HYST = -67, -77, 3      # dBm thresholds for levels 1 and 2


def lidar_bins(scan):
    """360 x uint16: nearest return (mm) per whole degree, 0 = nothing."""
    out = np.zeros(360, np.uint16)
    if scan is not None:
        a, d = scan
        idx = np.mod(np.round(a).astype(int), 360)
        d = np.clip(d, 0, 65535).astype(np.uint16)
        big = np.full(360, 65535, np.uint16)
        np.minimum.at(big, idx, d)
        out = np.where(big == 65535, 0, big).astype(np.uint16)
    return out.astype("<u2").tobytes()


def lidar_blob(scan, t, acc=None):
    ax, ay, az = (int(max(-128, min(127, round(v)))) for v in (acc or (0, 0, 0)))
    return lidar_bins(scan) + struct.pack("<dbbb", t, ax, ay, az)


def wifi_rssi(iface="wlan0"):
    """Signal level (dBm) of our association to B2, from /proc/net/wireless; None if not associated."""
    try:
        for line in open("/proc/net/wireless"):
            if line.strip().startswith(iface + ":"):
                v = float(line.split()[3].rstrip("."))
                return v if v < 0 else None
    except (OSError, ValueError, IndexError):
        pass
    return None


def pick_level(cur, rssi):
    if rssi is None:
        return 2
    want = 0 if rssi >= RSSI_FAIR else 1 if rssi >= RSSI_POOR else 2
    if want < cur:                                  # only step up once clearly better
        bound = RSSI_FAIR if want == 0 else RSSI_POOR
        if rssi < bound + HYST:
            return cur
    return want


class BulkLink:
    def __init__(self, view, cam, lid, host="192.168.4.1", port=5005, period=None, gap_s=0.003,
                 meta_fn=None, imu=None, level=None):
        self.view, self.cam, self.lid = view, cam, lid
        self.addr = (host, port)
        self.gap = gap_s
        self.meta_fn, self.imu = meta_fn, imu
        self.forced = level                         # None = auto from Wi-Fi signal
        self.level = 0 if level is None else int(level)
        self.rssi = None
        self.seq = {k: 0 for k in (KIND_CAMERA, KIND_THERMAL, KIND_LIDAR, KIND_META)}
        self.sent_bytes = 0
        self.status = "starting"
        self._stop = False
        self._due = {KIND_LIDAR: 0.0, KIND_META: 0.0, "image": 0.0}
        self._next_image = KIND_THERMAL

    def start(self):
        threading.Thread(target=self._run, name="bulklink", daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    # ---- payloads ----
    def _acc(self):
        try:
            s = self.imu.latest() if self.imu is not None else None
            return None if s is None else (s.ax, s.ay, s.az)
        except Exception:
            return None

    def _meta(self):
        m = {}
        if self.meta_fn is not None:
            try:
                m = dict(self.meta_fn() or {})
            except Exception as e:
                log.debug("meta_fn: %s", e)
        m.setdefault("t", round(time.time(), 2))
        m["lvl"], m["rssi"] = self.level, self.rssi
        if "acc" not in m:
            a = self._acc()
            if a is not None:
                m["acc"] = [int(round(v)) for v in a]
        return json.dumps(m, separators=(",", ":")).encode()

    def _image(self, kind):
        cv2, lv = self.view.cv2, LEVELS[self.level]
        if kind == KIND_THERMAL:
            th = self.cam.latest() if self.cam else None
            if th is None:
                return None
            small = cv2.resize(th, lv["th"], interpolation=cv2.INTER_AREA)
            return cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, lv["q"] + 10])[1].tobytes()
        try:
            jpg = self.view.renderer("rgb", True, {})()        # camera with person boxes / skeleton
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            small = cv2.resize(img, lv["cam"], interpolation=cv2.INTER_AREA)
            return cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, lv["q"]])[1].tobytes()
        except Exception as e:                                # no frame yet
            log.debug("camera frame: %s", e)
            return None

    def _send(self, sock, kind, blob):
        seq = self.seq[kind] = (self.seq[kind] + 1) & 0xFFFF
        n = max(1, -(-len(blob) // PAYLOAD))
        for i in range(n):
            part = blob[i * PAYLOAD:(i + 1) * PAYLOAD]
            sock.sendto(struct.pack("<BBBBHHH", 0xB7, 1, 3, kind, seq, i, n) + part, self.addr)
            self.sent_bytes += len(part)
            time.sleep(self.gap)

    # ---- scheduler ----
    def _step(self, sock, now):
        lv = LEVELS[self.level]
        if now >= self._due[KIND_LIDAR]:
            self._due[KIND_LIDAR] = now + lv["lidar"]
            self._send(sock, KIND_LIDAR, lidar_blob(self.lid.latest() if self.lid else None, now, self._acc()))
            return True
        if now >= self._due[KIND_META]:
            self._due[KIND_META] = now + lv["meta"]
            self._send(sock, KIND_META, self._meta())
            return True
        if now >= self._due["image"]:                 # camera and thermal take turns
            kind = self._next_image
            self._next_image = KIND_CAMERA if kind == KIND_THERMAL else KIND_THERMAL
            self._due["image"] = now + lv["image"] / 2
            blob = self._image(kind)
            if blob:
                self._send(sock, kind, blob)
            return True
        return False

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        last_log = last_lvl = time.time()
        while not self._stop:
            now = time.time()
            if now - last_lvl >= 2.0:
                last_lvl = now
                self.rssi = wifi_rssi()
                if self.forced is None:
                    new = pick_level(self.level, self.rssi)
                    if new != self.level:
                        log.info("bulk link level %d -> %d (rssi %s dBm)", self.level, new, self.rssi)
                        self.level = new
            try:
                busy = self._step(sock, now)
                self.status = "ok"
            except OSError as e:                              # not on RESCUE-LINK (yet)
                if self.status != f"down: {e}":
                    log.warning("bulk link down: %s", e)
                self.status = f"down: {e}"
                busy = False
                time.sleep(2)
            if now - last_log > 60:
                log.info("bulk link %s, level %d, rssi %s, %.1f KB sent in the last minute",
                         self.status, self.level, self.rssi, self.sent_bytes / 1024)
                self.sent_bytes, last_log = 0, now
            if not busy:
                time.sleep(0.005)
