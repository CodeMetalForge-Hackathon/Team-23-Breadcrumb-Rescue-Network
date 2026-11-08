"""Stable person ids across frames, in the LiDAR frame, with posture debounce and per-person alert timing.

Each frame: the Pi camera's people and the thermal camera's people (spotter.locate.Obs) are paired by bearing
(same person seen by both = one observation, src "CT"), then matched to the existing tracks by bearing and range.
A track is confirmed after 3 hits, and dropped after max_age_s without one. Posture is "U", "D" or None
(can't tell): D needs 5 of the last 7 frames to say D ("?" frames never count towards it) and holds while 3 do;
U needs 4 of 7.
"""
from collections import deque

from spotter.locate import Obs, angdiff

SEEN_S = 1.0          # a camera counts towards a track's src if it saw the person this recently


def pair_cameras(cam_obs, th_obs, merge_deg):
    """Pi-camera and thermal observations of the same person (bearings within merge_deg) -> one "CT" obs."""
    pairs = sorted((abs(angdiff(c.b, t.b)), i, j) for i, c in enumerate(cam_obs) for j, t in enumerate(th_obs))
    used_c, used_t, out = set(), set(), []
    for diff, i, j in pairs:
        if diff > merge_deg or i in used_c or j in used_t:
            continue
        used_c.add(i)
        used_t.add(j)
        c, t = cam_obs[i], th_obs[j]
        r, rs = (c.r, c.rs) if c.rs == "L" or t.r is None else (t.r, t.rs) if t.rs == "L" else (c.r, c.rs)
        ps = c.ps if t.ps in (None, "?", c.ps) else t.ps if c.ps in (None, "?") else "?"   # they disagree: "?"
        out.append(Obs("CT", c.b, r, rs, ps, max(c.c, t.c), c.det, c.u, c.why or t.why))
    out += [c for i, c in enumerate(cam_obs) if i not in used_c]
    out += [t for j, t in enumerate(th_obs) if j not in used_t]
    return out


class Track:
    def __init__(self, tid, obs, now):
        self.id, self.first_seen = tid, now
        self.b, self.r, self.rs, self.c, self.det, self.u = obs.b, obs.r, obs.rs, obs.c, obs.det, obs.u
        self.seen = {}                       # "C" / "T" -> last time that camera saw this person
        self.ps_hist = deque(maxlen=7)
        self.ps = None
        self.hits, self.last_seen = 0, now
        self.last_alert, self.alerted_ps = None, None
        self._take(obs, now)

    def _take(self, obs, now):
        self.hits += 1
        self.last_seen = now
        for s in obs.src:
            self.seen[s] = now
        self.c, self.det, self.u = obs.c, obs.det, obs.u
        self.ps_hist.append(obs.ps)
        d, u = self.ps_hist.count("D"), self.ps_hist.count("U")
        if d >= 5 or (self.ps == "D" and d >= 3):
            self.ps = "D"
        elif u >= 4 or (self.ps == "U" and u >= 2 and d < 3):
            self.ps = "U"
        else:
            self.ps = None

    def update(self, obs, now, alpha=0.5):
        self.b = (self.b + alpha * angdiff(obs.b, self.b)) % 360.0
        if obs.r is not None:
            if self.r is None or (obs.rs == "L" and self.rs != "L"):
                self.r, self.rs = obs.r, obs.rs              # a LiDAR range replaces a size guess outright
            elif obs.rs == self.rs:
                self.r += alpha * (obs.r - self.r)
            # else: a size guess while we have a LiDAR range (person went down below the scan plane): keep it
        self._take(obs, now)

    def src(self, now):
        return "".join(s for s in "CT" if now - self.seen.get(s, -1e9) <= SEEN_S)

    @property
    def confirmed(self):
        return self.hits >= 3


class Tracker:
    def __init__(self, gate_deg=12.0, gate_m=1.0, max_age_s=1.5, merge_deg=15.0):
        self.gate_deg, self.gate_m, self.max_age_s, self.merge_deg = gate_deg, gate_m, max_age_s, merge_deg
        self.tracks = []
        self._next = 1

    def _new_id(self):
        used = {t.id for t in self.tracks}
        while self._next in used:
            self._next = self._next % 99 + 1
        tid, self._next = self._next, self._next % 99 + 1
        return tid

    def update(self, cam_obs, th_obs, now):
        """Feed one frame's observations. Returns the live tracks."""
        obs = pair_cameras(list(cam_obs), list(th_obs), self.merge_deg)
        cand = []
        for ti, t in enumerate(self.tracks):
            for oi, o in enumerate(obs):
                db = abs(angdiff(o.b, t.b))
                dr = abs(o.r - t.r) if (o.r is not None and t.r is not None) else 0.0
                if db <= self.gate_deg and (dr <= self.gate_m or "S" in (o.rs, t.rs)):
                    cand.append((db + 10.0 * dr, ti, oi))
        used_t, used_o = set(), set()
        for _, ti, oi in sorted(cand):
            if ti in used_t or oi in used_o:
                continue
            used_t.add(ti)
            used_o.add(oi)
            self.tracks[ti].update(obs[oi], now)
        for oi, o in enumerate(obs):
            if oi not in used_o:
                self.tracks.append(Track(self._new_id(), o, now))
        self.tracks = [t for t in self.tracks if now - t.last_seen <= self.max_age_s]
        return self.tracks

    def due(self, now, cooldown_s):
        """Confirmed tracks seen this frame that should send an alert now: first sighting, just went down,
        or cooldown_s since the last alert."""
        out = []
        for t in self.tracks:
            if not t.confirmed or t.last_seen != now:
                continue
            if t.last_alert is None or (t.ps == "D" and t.alerted_ps != "D") or now - t.last_alert >= cooldown_s:
                t.last_alert, t.alerted_ps = now, t.ps
                out.append(t)
        return out

    def persons(self, now, limit=5, kp_max=3):
        """Confirmed tracks as compact dicts (meta / bulk kind 4): people down first, then nearest.
        The first kp_max also carry "kp" (keypoints_pct), so meta stays under ~900 B."""
        live = [t for t in self.tracks if t.confirmed]
        live.sort(key=lambda t: (t.ps != "D", t.r if t.r is not None else 99.0))
        out = []
        for i, t in enumerate(live[:limit]):
            p = {"id": t.id, "b": round(t.b), "r": None if t.r is None else round(t.r, 1), "rs": t.rs,
                 "src": t.src(now) or "-", "ps": t.ps or "?", "c": t.c}
            kp = keypoints_pct(t.det) if i < kp_max else None
            if kp is not None:
                p["kp"] = kp
            out.append(p)
        return out


def keypoints_pct(det, min_conf=0.5):
    """17 COCO keypoints -> 34 ints: x, y as percent 0..99 of the person's box, -1 where not visible."""
    kpts = getattr(det, "kpts", None)
    if kpts is None:
        return None
    w, h = max(det.x1 - det.x0, 1e-6), max(det.y1 - det.y0, 1e-6)
    out = []
    for x, y, conf in kpts:
        if conf < min_conf:
            out += [-1, -1]
        else:
            out += [min(max(int((x - det.x0) / w * 100), 0), 99), min(max(int((y - det.y0) / h * 100), 0), 99)]
    return out
