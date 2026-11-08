"""Floor plan for the command post: SLAM map from the bulk-link LiDAR, people from the spotter's meta (or from
PERSON alerts with bearing/range), relay node positions marked by the operator, and suggested relay spots.

Works without the mapping package too: the Pi then stays at the origin and only the live scan is drawn.
Map frame (shared with mapping.slam): first pose (0,0,0), th = heading in radians CLOCKWISE from the initial +y;
a sensor point (sx, sy) = (sin(a)*d, cos(a)*d) at pose (px,py,th) maps to
(px + cos(th)*sx + sin(th)*sy, py - sin(th)*sx + cos(th)*sy)  (th = pi/2: straight ahead is +x). grid img[iy, ix] covers
(ox + (ix+.5)*res, oy + (iy+.5)*res).
"""
import json
import logging
import math
import struct
import threading
import time
import zlib
from pathlib import Path

import numpy as np

log = logging.getLogger("mapview")

try:
    from mapping.slam import Slam2D
except Exception as e:                          # mapping not deployed yet: live-scan-only floor plan
    Slam2D = None
    log.info("no SLAM (%s)", e)
try:
    from mapping.relay import suggest_relays
except Exception:
    suggest_relays = None

PERSON_TTL = 180          # s a person stays on the map after the last sighting
PNG_EVERY = 1.0           # s between map image renders
RELAY_EVERY = 3.0
NODE_IDS = ["B2", "B1", "BASE"]
LUT = np.full(256, 22, np.uint8)               # unknown -> near background
LUT[:100] = 48                                 # free -> dark grey floor
LUT[160:] = 235                                # occupied -> bright wall


def png_gray(a):
    """Minimal 8-bit greyscale PNG encoder (no PIL / OpenCV on the laptop)."""
    a = np.ascontiguousarray(a, np.uint8)
    h, w = a.shape
    raw = b"".join(b"\x00" + a[i].tobytes() for i in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def sensor_to_map(pose, bearing_deg, range_m):
    px, py, th = pose
    a = np.radians(bearing_deg)
    sx, sy = np.sin(a) * range_m, np.cos(a) * range_m
    return px + math.cos(th) * sx + math.sin(th) * sy, py - math.sin(th) * sx + math.cos(th) * sy


class MapState:
    def __init__(self, nodes_file="logs/map_nodes.json"):
        self.lock = threading.Lock()
        self.nodes_file = Path(nodes_file)
        self.slam = None
        self.slam_error = None
        self._new_slam()
        self.pose = (0.0, 0.0, 0.0)
        self.path = []                       # [x, y]
        self.scan = []                       # latest scan, map frame, [[x, y], ...]
        self.persons = {}                    # id -> {...}
        self.nodes = {}
        self.relays = []
        self.png, self.png_info, self._png_t, self._relay_t = None, None, 0.0, 0.0
        self.scans, self.last_scan, self.hz = 0, None, 0.0
        self.scan_ok, self.reject_reason = True, None
        self.link = {}                       # lvl, rssi, tilt from the last meta
        try:
            self.nodes = json.loads(self.nodes_file.read_text())
        except (OSError, ValueError):
            pass

    def _new_slam(self):
        if Slam2D is None:
            return
        try:
            self.slam = Slam2D()
        except Exception as e:
            self.slam, self.slam_error = None, str(e)
            log.exception("Slam2D()")

    # ---- inputs ----
    def on_object(self, name, blob, parsed):
        if name == "lidar" and parsed is not None:
            self.add_scan(*parsed)
        elif name == "meta" and parsed is not None:
            self.add_meta(parsed)

    def add_scan(self, mm, t=None, acc=None, now=None):
        now = time.time() if now is None else now
        mm = np.asarray(mm, np.float64)
        idx = np.nonzero(mm > 0)[0]
        ang, dist = idx.astype(np.float64), mm[idx]
        with self.lock:
            if self.last_scan is not None:
                dt = max(now - self.last_scan, 1e-3)
                self.hz = 0.8 * self.hz + 0.2 / dt
            self.last_scan = now
            self.scans += 1
            if self.slam is not None and len(idx) >= 10:
                try:
                    res = self.slam.update(ang, dist, t=t, acc=acc)
                    self.pose = tuple(float(v) for v in self.slam.pose)
                    self.scan_ok = bool(res.get("ok", True)) if isinstance(res, dict) else True
                    self.reject_reason = None if self.scan_ok else str(res.get("reason") or "rejected")
                except Exception as e:
                    self.slam_error = f"update: {e}"
                    log.exception("slam.update")
            px, py, _ = self.pose
            if not self.path or math.hypot(px - self.path[-1][0], py - self.path[-1][1]) > 0.05:
                self.path.append([round(px, 3), round(py, 3)])
                if len(self.path) > 5000:
                    self.path = self.path[::2]
            keep = dist > 250                               # rig self-hits are closer than this
            xs, ys = sensor_to_map(self.pose, ang[keep], dist[keep] / 1000.0)
            self.scan = np.round(np.stack([xs, ys], 1), 3).tolist()
            if self.slam is not None and now - self._png_t >= PNG_EVERY:
                self._png_t = now
                self._render_png()
            if self.slam is not None and suggest_relays is not None and now - self._relay_t >= RELAY_EVERY:
                self._relay_t = now
                self._suggest()

    def add_meta(self, m, now=None):
        now = time.time() if now is None else now
        with self.lock:
            self.link = {k: m.get(k) for k in ("lvl", "rssi", "tilt") if k in m}
        for p in m.get("persons") or []:
            self.add_person(p.get("id"), p.get("b"), p.get("r"), ps=p.get("ps"), src=p.get("src"),
                            c=p.get("c"), rs=p.get("rs"), via="bulk", now=now)

    def add_alert(self, msg, now=None):
        """PERSON alert over LoRa: k (track id), b (deg), r (decimetres). Placed from the Pi's current pose."""
        if msg.get("t") != "PERSON" or msg.get("b") is None:
            return
        r = msg.get("r")
        self.add_person(msg.get("k", f"a{msg.get('id')}"), msg.get("b"), None if r is None else r / 10.0,
                        ps=msg.get("ps"), src=msg.get("s") or ("CT" if msg.get("th") else "C"),
                        c=msg.get("c"), via="lora", now=now)

    def add_person(self, pid, b, r, ps=None, src=None, c=None, rs=None, via="bulk", now=None):
        if pid is None or b is None:
            return
        now = time.time() if now is None else now
        with self.lock:
            p = self.persons.setdefault(str(pid), {"id": str(pid), "first": now})
            p.update(ps=ps or p.get("ps"), src=src or p.get("src"), c=c if c is not None else p.get("c"),
                     rs=rs, via=via, seen=now, b=b, r=r)
            if r:                                          # unknown range: keep the last known position
                x, y = sensor_to_map(self.pose, float(b), float(r))
                p["x"], p["y"] = round(float(x), 2), round(float(y), 2)
            elif "x" not in p:
                p["x"], p["y"] = None, None

    # ---- operator actions ----
    def set_node(self, nid, x=None, y=None, here=False, clear=False):
        if nid not in NODE_IDS:
            return False
        with self.lock:
            if clear:
                self.nodes.pop(nid, None)
            else:
                if here:
                    x, y = self.pose[0], self.pose[1]
                self.nodes[nid] = {"x": round(float(x), 2), "y": round(float(y), 2), "t": time.time()}
            try:
                self.nodes_file.parent.mkdir(parents=True, exist_ok=True)
                self.nodes_file.write_text(json.dumps(self.nodes))
            except OSError as e:
                log.warning("cannot save nodes: %s", e)
            self._relay_t = 0.0
        return True

    def reset(self):
        with self.lock:
            self._new_slam()
            self.pose, self.path, self.scan, self.persons, self.relays = (0.0, 0.0, 0.0), [], [], {}, []
            self.png, self.png_info = None, None

    # ---- outputs ----
    def _render_png(self):
        try:
            img, (ox, oy), res = self.slam.grid()
        except Exception as e:
            self.slam_error = f"grid: {e}"
            return
        img = np.asarray(img, np.uint8)
        known = np.nonzero(img != 127)
        if len(known[0]) == 0:
            return
        pad = 10
        y0, y1 = int(max(known[0].min() - pad, 0)), int(min(known[0].max() + pad + 1, img.shape[0]))
        x0, x1 = int(max(known[1].min() - pad, 0)), int(min(known[1].max() + pad + 1, img.shape[1]))
        ox, oy, res = float(ox), float(oy), float(res)
        crop = LUT[img[y0:y1, x0:x1]][::-1]                  # row 0 = largest y, as drawn on screen
        v = (self.png_info or {}).get("v", 0) + 1
        self.png = png_gray(crop)
        self.png_info = {"x0": ox + x0 * res, "y0": oy + y0 * res, "res": res, "w": x1 - x0, "h": y1 - y0, "v": v}

    def _suggest(self):
        try:
            img, origin, res = self.slam.grid()
            nodes = {k: (v["x"], v["y"]) for k, v in self.nodes.items()}
            path = [tuple(p) for p in self.path]
            out = suggest_relays(img, origin, res, nodes, path) or []
            rel = [r if isinstance(r, dict) else {"x": r[0], "y": r[1], "reason": r[2] if len(r) > 2 else ""}
                   for r in out][:3]
            self.relays = [{"x": round(float(r["x"]), 2), "y": round(float(r["y"]), 2), "reason": str(r.get("reason", ""))}
                           for r in rel]
        except Exception as e:
            self.slam_error = f"relay: {e}"
            log.debug("suggest_relays: %s", e)

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            for pid in [k for k, p in self.persons.items() if now - p["seen"] > PERSON_TTL]:
                del self.persons[pid]
            persons = [{**p, "age": round(now - p["seen"], 1)} for p in self.persons.values()]
            path = self.path if len(self.path) <= 800 else self.path[:: len(self.path) // 800 + 1] + [self.path[-1]]
            return {
                "slam": self.slam is not None, "slam_error": self.slam_error,
                "pose": [round(v, 3) for v in self.pose], "path": path, "scan": self.scan,
                "scan_ok": self.scan_ok, "reject_reason": self.reject_reason,
                "scan_age": None if self.last_scan is None else round(now - self.last_scan, 1),
                "hz": round(self.hz, 1), "scans": self.scans,
                "rejected": getattr(self.slam, "rejected", None),
                "persons": persons, "nodes": dict(self.nodes), "relays": self.relays,
                "png": self.png_info, "link": self.link,
            }

    def png_bytes(self):
        with self.lock:
            return self.png
