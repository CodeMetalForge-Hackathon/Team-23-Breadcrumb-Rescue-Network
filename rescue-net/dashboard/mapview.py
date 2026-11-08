"""Floor plan for the command post: SLAM map from the bulk-link LiDAR, people from the spotter's meta (or from
PERSON alerts with bearing/range), relay node positions marked by the operator, and suggested relay spots.

Works without the mapping package too: the Pi then stays at the origin and only the live scan is drawn.
Map frame (shared with mapping.slam): first pose (0,0,0), th = heading in radians CLOCKWISE from the initial +y;
a sensor point (sx, sy) = (sin(a)*d, cos(a)*d) at pose (px,py,th) maps to
(px + cos(th)*sx + sin(th)*sy, py - sin(th)*sx + cos(th)*sy)  (th = pi/2: straight ahead is +x). grid img[iy, ix] covers
(ox + (ix+.5)*res, oy + (iy+.5)*res).
"""
import collections
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
    import inspect
    _RELAY_KW = set(inspect.signature(suggest_relays).parameters) & {"walls", "measured"}
except Exception:
    suggest_relays, _RELAY_KW = None, set()
try:
    from mapping.floorplan import extract as extract_walls   # vector floor plan (claude_03): walls as segments
except Exception:
    extract_walls = None
try:
    from dashboard.cloud import Cloud            # 3D view (codex_02): add_scan(...), snapshot(persons)
except Exception as e:
    Cloud = None
    log.info("no 3D cloud (%s)", e)

PERSON_TTL = 180          # s a person stays on the map after the last sighting
MERGE_M, MERGE_IDLE = 0.8, 1.0   # a new track id this close to a person not updated for this long = same person
PERSON_TILT_MAX = 10.0    # deg (meta "tilt"): past this, a LiDAR range to a person is suspect (same gate as SLAM)
TILT_SAME_DEG, TILT_SAME_S = 15.0, 5.0   # tilted rig: a new track id this close in bearing, this soon = same person
LIDAR_MOUNT_TILT_DEG = 3.0   # LiDAR plane vs the rig, re-mounted 2026-11-08 07:15 EST (was 6): SLAM and 3D range-risk rule
PNG_EVERY = 1.0           # s between map image renders
RELAY_EVERY = 3.0
RSSI_NODE = "B2"          # the spotter's Wi-Fi RSSI (bulk meta "rssi") is measured to B2's RESCUE-LINK AP
RSSI_STEP_M, RSSI_EVERY = 0.2, 2.0   # keep a sample after this much movement or time (RSSI refreshes every 2 s)
WALLS_EVERY = 2.0
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


def _angdiff(a, b):
    """a - b wrapped to -180..180 degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


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
        self.walls, self._walls_t = [], 0.0
        self._times = collections.deque()
        self.paused = False                  # operator paused mapping (rig set down on the floor)
        self.scans, self.last_scan, self.hz = 0, None, 0.0
        self.scan_ok, self.reject_reason = True, None
        self.link = {}                       # lvl, rssi, tilt from the last meta
        self.tilted = False                  # rig tilted past PERSON_TILT_MAX at the last meta
        self.rssi_samples = []               # (x, y, rssi dBm, node) along the walk: measured link to RSSI_NODE
        self._rssi_t = 0.0
        self.cloud = None
        if Cloud is not None:
            try:
                self.cloud = Cloud(mount_tilt_deg=LIDAR_MOUNT_TILT_DEG)
            except Exception:
                log.exception("Cloud()")
        # node positions are not reloaded: a new run starts a new SLAM origin, so old positions would be wrong.
        # The file is only a record of the last placements.

    def _new_slam(self):
        if Slam2D is None:
            return
        try:
            self.slam = Slam2D(mount_tilt_deg=LIDAR_MOUNT_TILT_DEG)
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
            self._times.append(now)
            while self._times and now - self._times[0] > 5.0:
                self._times.popleft()
            self.hz = (len(self._times) - 1) / max(now - self._times[0], 1e-3) if len(self._times) > 1 else 0.0
            self.last_scan = now
            self.scans += 1
            if self.slam is not None and len(idx) >= 10:
                try:
                    res = self.slam.update(ang, dist, t=t, acc=acc)
                    self.pose = tuple(float(v) for v in self.slam.pose)
                    self.scan_ok = bool(res.get("ok", True)) if isinstance(res, dict) else True
                    self.reject_reason = None if self.scan_ok else str(res.get("reason") or "rejected")
                    if self.paused:
                        self.scan_ok, self.reject_reason = True, None    # paused on purpose, not a tracking failure
                except Exception as e:
                    self.slam_error = f"update: {e}"
                    log.exception("slam.update")
            if self.cloud is not None:
                try:
                    self.cloud.add_scan(self.pose, ang, dist, acc, self.scan_ok)
                except Exception:
                    log.exception("cloud.add_scan")
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
            if self.slam is not None and extract_walls is not None and now - self._walls_t >= WALLS_EVERY:
                self._walls_t = now
                self._walls()
            if self.slam is not None and suggest_relays is not None and now - self._relay_t >= RELAY_EVERY:
                self._relay_t = now
                self._suggest()

    def add_meta(self, m, now=None):
        now = time.time() if now is None else now
        with self.lock:
            self.link = {k: m.get(k) for k in ("lvl", "rssi", "tilt") if k in m}
            rssi = m.get("rssi")
            if rssi is not None and self.slam is not None and not self.paused and self.scan_ok:
                x, y = round(self.pose[0], 2), round(self.pose[1], 2)
                last = self.rssi_samples[-1] if self.rssi_samples else None
                if (last is None or math.hypot(x - last[0], y - last[1]) >= RSSI_STEP_M
                        or now - self._rssi_t >= RSSI_EVERY):
                    self.rssi_samples.append((x, y, float(rssi), RSSI_NODE))
                    self._rssi_t = now
                    if len(self.rssi_samples) > 4000:
                        self.rssi_samples = self.rssi_samples[::2]
        # Rig tilted past the SLAM gate: the LiDAR plane can hit the desk or floor in front of the person, so a
        # LiDAR range is suspect (one person became three dots at 3.0 / 1.2 / 0.6 m, 2026-11-08). add_person
        # then keeps the person on the map with their last level range (see there).
        tilt = m.get("tilt")
        self.tilted = self.reject_reason == "tilted" or (tilt is not None and tilt > PERSON_TILT_MAX)
        for p in m.get("persons") or []:
            self.add_person(p.get("id"), p.get("b"), p.get("r"), ps=p.get("ps"), src=p.get("src"),
                            c=p.get("c"), rs=p.get("rs"), kp=p.get("kp"), via="bulk", now=now)

    def add_alert(self, msg, now=None):
        """PERSON alert over LoRa: k (track id), b (deg), r (decimetres). Placed from the Pi's current pose."""
        if msg.get("t") != "PERSON" or msg.get("b") is None:
            return
        r = msg.get("r")
        self.add_person(msg.get("k", f"a{msg.get('id')}"), msg.get("b"), None if r is None else r / 10.0,
                        ps=msg.get("ps"), src=msg.get("s") or ("CT" if msg.get("th") else "C"),
                        c=msg.get("c"), via="lora", now=now)

    def add_person(self, pid, b, r, ps=None, src=None, c=None, rs=None, kp=None, via="bulk", now=None):
        if pid is None or b is None:
            return
        now = time.time() if now is None else now
        pid = str(pid)
        with self.lock:
            # Tilted rig and a LiDAR range (LoRa alerts carry no rs): the range may be the desk, not the person.
            # The person stays on the map: a new track id at the same bearing just now is the same person
            # re-tracked (no extra dot), and the range is the last one measured with the rig level. Only a
            # person first seen while tilted gets the suspect range, flagged "rq": "tilt", until the rig is level.
            unsure = bool(self.tilted and rs in ("L", None) and r)
            if unsure and pid not in self.persons:
                near = [(abs(_angdiff(float(b), float(q["b"]))), k) for k, q in self.persons.items()
                        if q.get("b") is not None and now - q["seen"] <= TILT_SAME_S]
                near = [(d, k) for d, k in near if d <= TILT_SAME_DEG]
                if near:
                    q = self.persons.pop(min(near)[1])
                    q["id"], q["merged"] = pid, q.get("merged", 0) + 1
                    self.persons[pid] = q
            if unsure and self.persons.get(pid, {}).get("r_ok"):
                r, unsure = self.persons[pid]["r_ok"], False
            if pid not in self.persons and r and not unsure:   # tracker gave a new id: take over a nearby idle entry
                x, y = sensor_to_map(self.pose, float(b), float(r))
                near = [(math.hypot(q["x"] - x, q["y"] - y), k) for k, q in self.persons.items()
                        if q.get("x") is not None and now - q["seen"] >= MERGE_IDLE]
                olds = [self.persons.pop(k) for d, k in sorted(near) if d <= MERGE_M]   # all idle duplicates there
                if olds:
                    q = olds[0]
                    q["id"], q["merged"] = pid, sum(o.get("merged", 0) + 1 for o in olds)
                    q["first"] = min(o["first"] for o in olds)
                    self.persons[pid] = q
            p = self.persons.setdefault(pid, {"id": pid, "first": now})
            p.update(ps=ps or p.get("ps"), src=src or p.get("src"), c=c if c is not None else p.get("c"),
                     rs=rs, via=via, seen=now, b=b, r=r)
            if r and not unsure and not (self.tilted and rs in ("L", None)):
                p["r_ok"] = r                              # measured with the rig level (or not by the LiDAR)
            if unsure:
                p["rq"] = "tilt"
            else:
                p.pop("rq", None)
            if kp is not None:                             # 17 COCO keypoints, % of the box (bulk meta only)
                p["kp"] = kp
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

    def set_paused(self, paused):
        """Pause mapping while the rig rests low (on the floor for the posture view); resume at the same spot."""
        with self.lock:
            self.paused = bool(paused)
            if self.slam is not None and hasattr(self.slam, "set_mapping_enabled"):
                self.slam.set_mapping_enabled(not self.paused)
        return self.paused

    def reset(self):
        with self.lock:
            self._new_slam()
            self.paused = False
            self.pose, self.path, self.scan, self.persons, self.relays = (0.0, 0.0, 0.0), [], [], {}, []
            self.nodes = {}                  # new origin: press D2 on each node again (BASE first, at the start)
            self.png, self.png_info, self.walls = None, None, []
            self.rssi_samples, self._rssi_t = [], 0.0
            if self.cloud is not None:
                try:
                    self.cloud.clear()
                except Exception:
                    log.exception("cloud.clear")

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

    def _walls(self):
        try:
            img, origin, res = self.slam.grid()
            out = extract_walls(img, origin, res) or {}
            self.walls = [[round(float(v), 2) for v in w[:4]] for w in out.get("walls", [])][:2000]
        except Exception as e:
            self.slam_error = f"walls: {e}"
            log.debug("extract_walls: %s", e)

    def _suggest(self):
        try:
            img, origin, res = self.slam.grid()
            nodes = {k: (v["x"], v["y"]) for k, v in self.nodes.items()}
            path = [tuple(p) for p in self.path]
            kw = {}
            if "walls" in _RELAY_KW and self.walls:
                kw["walls"] = [list(w) for w in self.walls]
            if "measured" in _RELAY_KW and self.rssi_samples:
                kw["measured"] = list(self.rssi_samples)
            out = suggest_relays(img, origin, res, nodes, path, **kw) or []
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
            rs = self.rssi_samples if len(self.rssi_samples) <= 800 else self.rssi_samples[:: len(self.rssi_samples) // 800 + 1]
            return {
                "slam": self.slam is not None, "slam_error": self.slam_error,
                "pose": [round(v, 3) for v in self.pose], "path": path, "scan": self.scan,
                "scan_ok": self.scan_ok, "reject_reason": self.reject_reason,
                "scan_age": None if self.last_scan is None else round(now - self.last_scan, 1),
                "hz": round(self.hz, 1), "scans": self.scans,
                "rejected": getattr(self.slam, "rejected", None),
                "persons": persons, "nodes": dict(self.nodes), "relays": self.relays, "walls": self.walls,
                "png": self.png_info, "link": self.link, "paused": self.paused,
                "rssi": [[x, y, r] for x, y, r, _ in rs], "rssi_node": RSSI_NODE,   # measured signal along the walk
            }

    def cloud_snapshot(self):
        if self.cloud is None:
            return None
        now = time.time()
        with self.lock:
            persons = [{**p, "age": round(now - p["seen"], 1)} for p in self.persons.values()
                       if now - p["seen"] <= PERSON_TTL]
            pose, path = self.pose, list(self.path)
        try:
            return self.cloud.snapshot(persons=persons, pose=pose, path=path, nodes=dict(self.nodes))
        except TypeError:
            return self.cloud.snapshot(persons)

    def png_bytes(self):
        with self.lock:
            return self.png
