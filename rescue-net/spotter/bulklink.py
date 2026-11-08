"""Bulk uplink over our own relay chain (no venue Wi-Fi, no internet):

  Pi --Wi-Fi (B2's hidden RESCUE-LINK AP, UDP 192.168.4.1:5005)--> B2 --ESP-NOW--> B1 --> BASE --> dashboard

Every `period` seconds: a LiDAR outline (720 B), a thermal JPEG and a camera JPEG (each ~5-15 KB).
Chunk = B7 01 <hop=3> <kind> <seq u16> <idx u16> <cnt u16> + payload (<= 230 B), all little endian.
Alerts do not use this: they keep going over USB -> SP1 -> LoRa.
"""
import logging
import socket
import struct
import threading
import time

import numpy as np

log = logging.getLogger("bulklink")

KIND_CAMERA, KIND_THERMAL, KIND_LIDAR = 1, 2, 3
PAYLOAD = 230


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


class BulkLink:
    def __init__(self, view, cam, lid, host="192.168.4.1", port=5005, period=2.0, gap_s=0.008,
                 cam_size=(320, 240), cam_q=45, th_size=(320, 256), th_q=55):
        self.view, self.cam, self.lid = view, cam, lid
        self.addr = (host, port)
        self.period, self.gap = period, gap_s
        self.cam_size, self.cam_q, self.th_size, self.th_q = cam_size, cam_q, th_size, th_q
        self.seq = {KIND_CAMERA: 0, KIND_THERMAL: 0, KIND_LIDAR: 0}
        self.sent_bytes = 0
        self.status = "starting"
        self._stop = False

    def start(self):
        threading.Thread(target=self._run, name="bulklink", daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    def _items(self):
        cv2 = self.view.cv2
        yield KIND_LIDAR, lidar_bins(self.lid.latest() if self.lid else None)
        th = self.cam.latest() if self.cam else None
        if th is not None:
            small = cv2.resize(th, self.th_size, interpolation=cv2.INTER_AREA)
            yield KIND_THERMAL, cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, self.th_q])[1].tobytes()
        try:
            jpg = self.view.renderer("rgb", True, {})()        # camera with person boxes / skeleton
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            small = cv2.resize(img, self.cam_size, interpolation=cv2.INTER_AREA)
            yield KIND_CAMERA, cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, self.cam_q])[1].tobytes()
        except Exception as e:                                # no frame yet
            log.debug("camera frame: %s", e)

    def _send(self, sock, kind, blob):
        seq = self.seq[kind] = (self.seq[kind] + 1) & 0xFFFF
        n = max(1, -(-len(blob) // PAYLOAD))
        for i in range(n):
            part = blob[i * PAYLOAD:(i + 1) * PAYLOAD]
            sock.sendto(struct.pack("<BBBBHHH", 0xB7, 1, 3, kind, seq, i, n) + part, self.addr)
            self.sent_bytes += len(part)
            time.sleep(self.gap)

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        last_log = time.time()
        while not self._stop:
            t0 = time.time()
            try:
                for kind, blob in self._items():
                    self._send(sock, kind, blob)
                self.status = "ok"
            except OSError as e:                              # not on RESCUE-LINK (yet)
                if self.status != f"down: {e}":
                    log.warning("bulk link down: %s", e)
                self.status = f"down: {e}"
                time.sleep(3)
            if time.time() - last_log > 60:
                log.info("bulk link %s, %.1f KB sent in the last minute", self.status, self.sent_bytes / 1024)
                self.sent_bytes, last_log = 0, time.time()
            time.sleep(max(0.0, self.period - (time.time() - t0)))
