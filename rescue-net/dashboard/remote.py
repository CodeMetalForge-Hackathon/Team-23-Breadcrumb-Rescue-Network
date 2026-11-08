"""Reassemble bulk-link objects (camera JPEG, thermal JPEG, LiDAR scan, meta JSON) from BASE's "$B" lines.

BASE prints one line per chunk:  $B <kind> <seq> <idx> <cnt> <base64 payload>
kind 1 = camera JPEG, 2 = thermal JPEG,
     3 = LiDAR: 360 x uint16 little-endian, mm per degree (0 = no return); v2 adds t (float64) + ax, ay, az (int8),
     4 = meta: UTF-8 JSON from the spotter (people with bearing/range, link level, rssi, acc, ...).
on_object(name, blob, parsed) is called (outside the lock) for every completed object.
"""
import base64
import json
import struct
import threading
import time

KINDS = {1: "camera", 2: "thermal", 3: "lidar", 4: "meta"}


def parse_lidar(blob):
    """-> (mm list of 360, t or None, acc tuple or None), or None if malformed."""
    if len(blob) < 720:
        return None
    mm = list(struct.unpack("<360H", blob[:720]))
    t, acc = None, None
    if len(blob) >= 731:
        t, ax, ay, az = struct.unpack("<dbbb", blob[720:731])
        acc = None if (ax, ay, az) == (0, 0, 0) else (ax, ay, az)
    return mm, t, acc


class Remote:
    def __init__(self, on_object=None):
        self._lock = threading.Lock()
        self._parts = {}        # kind -> [seq, cnt, {idx: bytes}]
        self._done = {}         # kind -> (seq, bytes, t)
        self._stats = {k: {"ok": 0, "lost": 0} for k in KINDS}
        self._chunks = 0
        self._rate = []         # (t, nbytes) for the last 10 s
        self.meta = None        # last meta dict
        self.on_object = on_object

    def add(self, text, now=None):
        """Feed one "$B ..." line; returns True if it completed an object."""
        now = time.time() if now is None else now
        try:
            _, kind, seq, idx, cnt, b64 = text.strip().split(" ", 5)
            kind, seq, idx, cnt = int(kind), int(seq), int(idx), int(cnt)
            data = base64.b64decode(b64)
        except ValueError:
            return False
        if kind not in KINDS or not 0 <= idx < cnt:
            return False
        with self._lock:
            self._chunks += 1
            self._rate.append((now, len(data)))
            self._rate = [r for r in self._rate if now - r[0] < 10]
            cur = self._parts.get(kind)
            if cur is None or cur[0] != seq or cur[1] != cnt:
                if cur is not None:
                    self._stats[kind]["lost"] += 1          # previous object never completed
                cur = self._parts[kind] = [seq, cnt, {}]
            cur[2][idx] = data
            if len(cur[2]) < cnt:
                return False
            blob = b"".join(cur[2][i] for i in range(cnt))
            del self._parts[kind]
            self._done[kind] = (seq, blob, now)
            self._stats[kind]["ok"] += 1
        parsed = None
        if kind == 3:
            parsed = parse_lidar(blob)
        elif kind == 4:
            try:
                parsed = json.loads(blob.decode("utf-8"))
                parsed["_rx"] = now
                self.meta = parsed
            except (ValueError, UnicodeDecodeError):
                parsed = None
        if self.on_object is not None:
            try:
                self.on_object(KINDS[kind], blob, parsed)
            except Exception:                               # never let a consumer break the reader
                import logging
                logging.getLogger("remote").exception("on_object")
        return True

    def image(self, name):
        kind = {v: k for k, v in KINDS.items()}.get(name)
        with self._lock:
            d = self._done.get(kind)
        return None if d is None else d[1]

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        out = {}
        with self._lock:
            for kind, name in KINDS.items():
                d = self._done.get(kind)
                item = {"seq": None, "age": None, "bytes": 0, **self._stats[kind]}
                if d is not None:
                    item.update(seq=d[0], age=round(now - d[2], 1), bytes=len(d[1]))
                    if kind == 3:
                        p = parse_lidar(d[1])
                        if p is not None:
                            item["mm"], item["t"], item["acc"] = p
                pend = self._parts.get(kind)
                item["partial"] = None if pend is None else f"{len(pend[2])}/{pend[1]}"
                out[name] = item
            span = 10.0
            out["kbps"] = round(sum(n for t, n in self._rate if now - t < span) * 8 / span / 1000, 1)
            out["chunks"] = self._chunks
        m = self.meta
        out["meta"] = None if m is None else {**m, "age": round(now - m.get("_rx", now), 1)}
        return out
