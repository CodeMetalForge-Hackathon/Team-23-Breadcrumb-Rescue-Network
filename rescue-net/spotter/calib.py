"""Camera -> LiDAR calibration: where each camera points (yaw) and how wide it sees (HFOV), from a person
walking in front of the rig.

Record (on the Pi, spotter running): set "calib_record_s": 45 in config.json. The spotter writes every LiDAR
revolution with the people both cameras see to ~/logs/calib-<time>.jsonl.gz; set it back to 0 afterwards.
Have one person stand about 2 m away in an open spot: out of view 5 s, centre 10 s, left edge 10 s, right edge 10 s.

Or, on the Pi:  python3 -m spotter.calib rec standing 25   (sets calib_tag / calib_record_s, waits, resets them,
prints a summary). Each line also has the keypoints, live posture, range and accelerometer, for spotter.posture.

Fit (anywhere):  python3 -m spotter.calib ~/logs/calib-XXXX.jsonl.gz
The person's LiDAR bearing is where the scan is closer than the background (90th percentile per degree);
the fit finds yaw, HFOV and direction per camera so that bearing(box centre) matches it.
"""
import glob
import gzip
import json
import logging
import math
import os
import re
import sys
import time

import numpy as np

log = logging.getLogger("calib")


class Recorder:
    """Starts when calib_record_s changes from 0 to N > 0, writes N seconds, then closes."""

    def __init__(self, folder="~/logs"):
        self.folder = os.path.expanduser(folder)
        self.prev, self.f, self.until, self.path, self.last_scan, self.n = 0, None, 0.0, None, None, 0

    def step(self, cfg, cam_obs, th_obs, scan, imu=None):
        """cam_obs / th_obs: spotter.locate.Obs; imu: spotter.imu.ImuSample or None."""
        val = float(cfg.get("calib_record_s", 0) or 0)
        now = time.time()
        if val > 0 and self.prev <= 0 and self.f is None:
            os.makedirs(self.folder, exist_ok=True)
            tag = "".join(ch for ch in str(cfg.get("calib_tag", "")) if ch.isalnum() or ch in "-_")
            self.path = os.path.join(self.folder, "calib-" + (tag + "-" if tag else "")
                                     + time.strftime("%Y%m%d-%H%M%S.jsonl.gz"))
            self.f, self.until, self.n = gzip.open(self.path, "wt"), now + val, 0
            log.info("calibration: recording %.0f s to %s", val, self.path)
        self.prev = val
        if self.f is None:
            return
        if scan is not None and scan is not self.last_scan:           # one line per LiDAR revolution
            self.last_scan = scan
            a, d = scan
            def boxes(obs):
                return [[round(v, 4) for v in (o.det.x0, o.det.y0, o.det.x1, o.det.y1, o.det.score)] for o in obs]

            def extra(obs):
                return [{"k": None if getattr(o.det, "kpts", None) is None else
                         np.round(np.asarray(o.det.kpts, float), 3).tolist(),
                         "ps": o.ps, "b": round(o.b, 1), "r": None if o.r is None else round(o.r, 2), "rs": o.rs}
                        for o in obs]
            line = {"t": round(now, 3), "cam": boxes(cam_obs), "th": boxes(th_obs or ()),
                    "camk": extra(cam_obs), "thk": extra(th_obs or ()),
                    "acc": None if imu is None else [round(imu.ax, 2), round(imu.ay, 2), round(imu.az, 2)],
                    "ang": np.round(np.asarray(a) * 10).astype(int).tolist(),
                    "mm": np.round(np.asarray(d)).astype(int).tolist()}
            self.f.write(json.dumps(line, separators=(",", ":")) + "\n")
            self.n += 1
        if now >= self.until:
            self.f.close()
            self.f = None
            log.info("calibration: recording done, %d revolutions in %s", self.n, self.path)


# ---------------------------------------------------------------------------------------------------- fit

def load(path):
    with gzip.open(path, "rt") as f:
        return [json.loads(line) for line in f if line.strip()]


def bins_of(frame):
    """One revolution -> 360 ranges (m) by whole degree, nearest return, nan = none."""
    out = np.full(360, np.nan)
    a = np.mod(np.round(np.asarray(frame["ang"]) / 10).astype(int), 360)
    d = np.asarray(frame["mm"], float) / 1000
    for i, r in zip(a, d):
        if r >= 0.25 and not (r >= out[i]):
            out[i] = r
    return out


def person_bearing(bins, bg, min_gap=0.3, max_m=6.0, behind=(110, 250)):
    """Bearing (deg) of the largest run of degrees that are clearly closer than the background, or None.
    Degrees behind the rig (where it hits itself and the people working at it) are not searched."""
    fg = (bins < bg - min_gap) & (bins < max_m)
    fg = np.where(np.isnan(bins), False, fg)
    fg[behind[0]:behind[1] + 1] = False
    if not fg.any():
        return None
    if fg.all():
        return None
    start = int(np.argmin(fg))                       # rotate so index 0 is background (no run wraps)
    idx = (np.arange(360) + start) % 360
    runs, cur = [], []
    for i in idx:
        if fg[i]:
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    best = max(runs, key=len)
    if len(best) < 2:
        return None
    rad = np.radians(best)
    return float(np.degrees(np.arctan2(np.sin(rad).mean(), np.cos(rad).mean())) % 360)


def pairs(frames):
    """-> {"C": [(u, bearing)], "T": [...]} from revolutions where exactly one person is seen."""
    allbins = np.array([bins_of(f) for f in frames])
    import warnings
    with warnings.catch_warnings():                  # degrees with no return at all: nan background
        warnings.simplefilter("ignore", RuntimeWarning)
        bg = np.nanpercentile(allbins, 90, axis=0)
    out = {"C": [], "T": []}
    for f, bins in zip(frames, allbins):
        b = None
        for key, src in (("cam", "C"), ("th", "T")):
            if len(f[key]) != 1:
                continue
            if b is None:
                b = person_bearing(bins, bg)
                if b is None:
                    break
            x0, _, x1, _, _ = f[key][0]
            out[src].append(((x0 + x1) / 2, b))
    return out


def fit(pts, hfov_fixed=None, inlier_deg=6.0):
    """Best (yaw, hfov, direction, rms_deg, inlier fraction) for bearing = yaw + dir * atan((u - .5) * 2 tan(hfov/2)).
    Robust: for each candidate, yaw is the mode of (bearing - offset); score = pairs within inlier_deg of it."""
    from spotter.locate import angdiff
    u = np.array([p[0] for p in pts])
    b = np.array([p[1] for p in pts])
    spread = np.percentile(u, 95) - np.percentile(u, 5)
    hfovs = [hfov_fixed] if (hfov_fixed or spread < 0.25) else np.arange(20, 120.01, 0.5)
    best = None
    for direction in (1, -1):
        for hfov in hfovs:
            off = direction * np.degrees(np.arctan((u - 0.5) * 2 * math.tan(math.radians(hfov) / 2)))
            r = (b - off) % 360
            hist = np.bincount(np.round(r).astype(int) % 360, minlength=360)
            win = np.convolve(np.concatenate([hist[-6:], hist, hist[:6]]), np.ones(13), "valid")[:360]
            yaw0 = float(np.argmax(win))
            inl = np.abs(angdiff(r, yaw0)) <= inlier_deg
            if not inl.any():
                continue
            rad = np.radians(r[inl])
            yaw = math.degrees(math.atan2(np.sin(rad).mean(), np.cos(rad).mean())) % 360
            res = angdiff(r, yaw)
            score = float(np.clip(1 - (res / inlier_deg) ** 2, 0, None).sum())   # more inliers, and tighter
            if best is None or score > best[0]:
                inl = np.abs(res) <= inlier_deg
                best = (score, yaw, float(hfov), direction, float(np.sqrt(np.mean(res[inl] ** 2))), inl.mean())
    _, yaw, hfov, direction, rms, frac = best
    return yaw, hfov, direction, rms, float(frac)


def main(path, default_hfov=None):
    frames = load(path)
    print(f"{len(frames)} revolutions in {path}")
    out = {}
    for src, hkey, ykey, dkey in (("C", "cam_hfov", "cam_yaw", "cam_dir"), ("T", "th_hfov", "th_yaw", "th_dir")):
        pts = pairs(frames)[src]
        if len(pts) < 10:
            print(f"{src}: only {len(pts)} usable revolutions, skipped")
            continue
        u = [p[0] for p in pts]
        yaw, hfov, direction, rms, frac = fit(pts, (default_hfov or {}).get(src))
        print(f"{src}: {len(pts)} pairs, u {min(u):.2f}..{max(u):.2f}  ->  yaw {yaw:.1f}  hfov {hfov:.1f}  "
              f"dir {direction:+d}  rms {rms:.1f} deg over {frac:.0%} inliers")
        out.update({ykey: round(yaw, 1), hkey: round(hfov, 1), dkey: direction})
    print(json.dumps(out))
    return out


# ---------------------------------------------------------------------------------------------------- record

def set_config(path, **kv):
    """Change values in config.json in place (keeps the file's layout), atomically: the spotter hot-reloads it."""
    path = str(path)
    with open(path) as f:
        text = f.read()
    for k, v in kv.items():
        val = json.dumps(v)
        pat = re.compile(r'("%s"\s*:\s*)("(?:[^"\\]|\\.)*"|[^,\n}]+)' % re.escape(k))
        if pat.search(text):
            text = pat.sub(lambda m: m.group(1) + val, text, count=1)
        else:
            i = text.rstrip().rfind("}")
            text = text[:i].rstrip() + f',\n  "{k}": {val}\n' + text[i:]
    json.loads(text)                                   # never leave a broken file behind
    with open(path + ".tmp", "w") as f:
        f.write(text)
    os.replace(path + ".tmp", path)


def summary(path):
    """Quick check of one recording: frames, how often each camera saw someone, live posture votes, gravity."""
    frames = load(path)
    out = {"file": os.path.basename(path), "frames": len(frames)}
    for key in ("camk", "thk"):
        obs = [o for f in frames for o in f.get(key, [])]
        out[key[:-1] + "_seen"] = sum(1 for f in frames if f.get(key))
        out[key[:-1] + "_ps"] = {p: sum(1 for o in obs if o["ps"] == p) for p in ("U", "D", "?")}
    acc = np.array([f["acc"] for f in frames if f.get("acc")])
    if len(acc):
        out["acc_mean"] = np.round(acc.mean(0), 2).tolist()
        out["acc_min"], out["acc_max"] = np.round(acc.min(0), 1).tolist(), np.round(acc.max(0), 1).tolist()
    return out


def record(tag, seconds, cfg_path=None, folder="~/logs"):
    """Run one tagged recording on the Pi (spotter running) and print its summary."""
    from spotter.config import DEFAULT_PATH
    cfg_path = cfg_path or DEFAULT_PATH
    tag = "".join(ch for ch in tag if ch.isalnum() or ch in "-_")
    folder, t0 = os.path.expanduser(folder), time.time()
    set_config(cfg_path, calib_tag=tag, calib_record_s=0)
    time.sleep(1.5)                                    # the spotter checks the file twice a second
    set_config(cfg_path, calib_record_s=seconds)
    print(f"recording {tag} for {seconds} s ...", flush=True)
    try:
        time.sleep(seconds + 2)
    finally:
        set_config(cfg_path, calib_record_s=0, calib_tag="")
    new = [f for f in glob.glob(os.path.join(folder, f"calib-{tag}-*.jsonl.gz")) if os.path.getmtime(f) >= t0]
    if not new:
        print("no recording written: is rescue-spotter running?")
        return None
    s = summary(max(new, key=os.path.getmtime))
    print(json.dumps(s))
    return s


if __name__ == "__main__":
    # python3 -m spotter.calib FILE               fit camera yaw / HFOV
    # python3 -m spotter.calib rec TAG SECONDS    tagged recording (on the Pi)
    # python3 -m spotter.calib sum FILE           summary of a recording
    if sys.argv[1] == "rec":
        record(sys.argv[2], float(sys.argv[3]))
    elif sys.argv[1] == "sum":
        print(json.dumps(summary(sys.argv[2])))
    else:
        main(sys.argv[1])
