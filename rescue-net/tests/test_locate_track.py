import json
import math

import numpy as np
import pytest

from common.protocol import encode_line
from spotter.config import DEFAULTS
from spotter.detector import PoseDetection
from spotter.imu import REF, Imu, tilt_deg, to_signed6
from spotter.locate import Obs, bearing, lidar_range, observe, size_range
from spotter.tracker import Tracker, pair_cameras


def cfg(**over):
    c = dict(DEFAULTS)
    c.update(cam_hfov=66.0, cam_yaw=0.0, cam_dir=1, th_hfov=50.0, th_yaw=0.0, th_dir=1)   # not the calibration
    c.update(over)
    return c


def det(x0, y0, x1, y1, score=0.8, lying=False):
    k = np.zeros((17, 3), np.float32)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if lying:
        k[5], k[6], k[11], k[12] = (x0, cy, 1), (x0, cy, 1), (x1, cy, 1), (x1, cy, 1)
    else:
        k[5], k[6], k[11], k[12] = (cx, y0, 1), (cx, y0, 1), (cx, y1, 1), (cx, y1, 1)
    return PoseDetection("person", score, x0, y0, x1, y1, k)


def scan_with(objects, wall_m=None, rig=True):
    """360-ish returns: (deg_from, deg_to, metres) objects, optional wall everywhere else, rig hits at 120-240."""
    a = np.arange(0, 360, 0.5, dtype=np.float32)
    d = np.full(a.shape, 0.0 if wall_m is None else wall_m * 1000, np.float32)
    for lo, hi, m in objects:
        sel = (a >= lo % 360) & (a <= hi % 360) if lo % 360 <= hi % 360 else (a >= lo % 360) | (a <= hi % 360)
        d[sel] = m * 1000
    if rig:
        d[(a >= 120) & (a <= 240)] = 120.0
    keep = d > 0
    return a[keep], d[keep]


# --- bearing / range -------------------------------------------------------------------------------------------

def test_bearing_centre_edges_and_direction():
    assert bearing(0.5, 66, 10) == pytest.approx(10)
    assert bearing(1.0, 66, 0) == pytest.approx(33)
    assert bearing(0.0, 66, 0) == pytest.approx(360 - 33)
    assert bearing(1.0, 66, 0, -1) == pytest.approx(360 - 33)


def test_lidar_range_nearest_surface_ignores_wall_and_rig():
    s = scan_with([(28, 32, 2.0)], wall_m=4.0)
    assert lidar_range(s, 30) == pytest.approx(2.0)
    assert lidar_range(s, 180) is None                    # only the rig itself there (< 0.25 m)
    assert lidar_range(None, 30) is None
    assert lidar_range(scan_with([(359, 1, 1.5)], wall_m=5.0), 0.5) == pytest.approx(1.5)   # wraps at 0


def test_observe_upright_uses_lidar():
    d = det(0.45, 0.2, 0.55, 0.9)                         # centre of the image, about 2 m away by size
    o = observe(d, "C", scan_with([(-3, 3, 2.1)], wall_m=5.0), cfg())
    assert o.src == "C" and o.rs == "L" and o.r == pytest.approx(2.1) and o.ps == "U"
    assert abs(o.b) < 1 or abs(o.b - 360) < 1


def test_observe_person_lying_below_scan_plane_falls_back_to_size():
    d = det(0.3, 0.6, 0.7, 0.8, lying=True)
    o = observe(d, "C", scan_with([], wall_m=8.0), cfg())   # LiDAR only sees the far wall
    assert o.ps == "D" and o.rs == "S" and 0.5 < o.r < 4.0


def test_size_range_roughly_right():
    # an upright person 1.7 m tall at 3 m fills 1.7 / (2 * 3 * tan(vfov/2)) of the image height
    tx = math.tan(math.radians(66) / 2)
    hfrac = 1.7 / (2 * 3 * tx * 9 / 16)
    r = size_range(det(0.45, 0.5 - hfrac / 2, 0.55, 0.5 + hfrac / 2), 66, 9 / 16, "U")
    assert r == pytest.approx(3.0, rel=0.01)


# --- tracking --------------------------------------------------------------------------------------------------

def ob(b, r=2.0, src="C", ps="U", c=80):
    return Obs(src, b, r, "L" if r is not None else None, ps, c, object(), 0.5)


def test_pair_cameras_merges_same_person_only():
    out = pair_cameras([ob(10), ob(90)], [ob(14, src="T")], merge_deg=15)
    assert sorted(o.src for o in out) == ["C", "CT"]
    assert len(pair_cameras([ob(10)], [ob(60, src="T")], merge_deg=15)) == 2


def test_track_ids_stable_and_second_person_new_id():
    tr = Tracker()
    for i in range(5):
        tr.update([ob(10 + i)], [], i * 0.1)
    assert [t.id for t in tr.tracks] == [1] and tr.tracks[0].confirmed
    tr.update([ob(15), ob(80, r=3.0)], [], 0.5)
    assert sorted(t.id for t in tr.tracks) == [1, 2]
    tr.update([ob(81, r=3.0)], [], 2.5)                  # person 1 gone for > 1.5 s
    assert [t.id for t in tr.tracks] == [2]


def test_posture_debounce_and_alert_timing():
    tr = Tracker()
    fired, states = [], []
    seq = ["U", "U", "U", "U", "D", "?", "D", "D", "?", "D", "D", "D", "D"]
    for i, ps in enumerate(seq):
        now = i * 0.1
        tr.update([ob(20, ps=ps)], [], now)
        states.append(tr.tracks[0].ps)
        fired += [(i, t.ps) for t in tr.due(now, cooldown_s=10)]
    # confirmed at the 3rd frame with posture still unknown; U after 4 U votes; D only once 5 of 7 say D
    assert fired[0] == (2, None) and states[3] == "U"
    first_d = states.index("D")
    assert seq[max(0, first_d - 6):first_d + 1].count("D") >= 5
    assert fired[-1] == (first_d, "D") and len(fired) == 2     # going down fires at once


def test_question_marks_never_make_down():
    tr = Tracker()
    for i, ps in enumerate(["D", "?", "D", "?", "D", "?", "D", "?"]):
        tr.update([ob(20, ps=ps)], [], i * 0.1)
    assert tr.tracks[0].ps is None
    assert tr.persons(0.7)[0]["ps"] == "?"


# --- posture (3 states) ----------------------------------------------------------------------------------------

def body(x0, y0, x1, y1, torso_deg, roll=0.0, x_scale=16 / 9):
    """Pose detection whose shoulders -> hips line is torso_deg from vertical (in the image, after roll)."""
    k = np.zeros((17, 3), np.float32)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    a = math.radians(torso_deg + roll)
    L = 0.25
    dx, dy = math.sin(a) * L / x_scale, math.cos(a) * L
    k[5] = k[6] = (cx - dx / 2, cy - dy / 2, 1)
    k[11] = k[12] = (cx + dx / 2, cy + dy / 2, 1)
    return PoseDetection("person", 0.9, x0, y0, x1, y1, k)


def test_posture_three_states():
    from spotter.posture import classify
    c = cfg()
    assert classify(body(0.3, 0.2, 0.5, 0.8, 5), c)[0] == "U"
    assert classify(body(0.3, 0.2, 0.5, 0.8, 45), c)[0] == "?"           # in between: can't tell
    assert classify(body(0.2, 0.5, 0.7, 0.7, 85), c)[0] == "D"           # lying, fully in view
    # cut by the bottom edge with no keypoints: wide box says nothing
    assert classify(PoseDetection("person", 0.9, 0.2, 0.5, 0.8, 1.0, np.zeros((17, 3))), c)[0] == "?"


def test_posture_vetoes_only_doubtful_down():
    from spotter.posture import classify
    c = cfg()
    cut = body(0.0, 0.3, 0.6, 1.0, 65)                    # 65 deg, box cut by the image edge
    assert classify(cut, c)[0] == "D"
    assert classify(cut, c, lidar_body=True)[0] == "?"    # the body crosses the scan plane
    assert classify(cut, c, extent_m=1.1)[0] == "?"       # 1.1 m visible: not lying flat
    sure = body(0.2, 0.5, 0.7, 0.7, 85)                   # clearly horizontal, fully in view
    assert classify(sure, c, lidar_body=True)[0] == "D"   # a wall right behind them does not hide them
    assert classify(body(0.3, 0.1, 0.5, 0.9, 45), c, extent_m=1.2)[0] == "U"


def test_posture_roll_correction():
    from spotter.imu import ImuSample, roll_deg
    from spotter.posture import classify
    c = cfg()
    tilted = body(0.3, 0.2, 0.5, 0.8, 0, roll=40)          # standing, rig rolled 40 deg
    assert classify(tilted, c)[0] == "?"
    assert classify(tilted, c, roll_deg=40)[0] == "U"
    s = ImuSample(0, 21.0 * math.sin(math.radians(30)) + 1.45, 1.33, 21.0 * math.cos(math.radians(30)), 30)
    assert roll_deg(s, "x") == pytest.approx(30, abs=4) and roll_deg(s, "-x") == pytest.approx(-30, abs=4)
    assert roll_deg(s, "") == 0 and roll_deg(None, "x") == 0


def test_cooldown_repeats():
    tr = Tracker()
    n = 0
    for i in range(30):
        tr.update([ob(20)], [], i * 0.5)
        n += len(tr.due(i * 0.5, cooldown_s=5))
    assert n == 3                                         # t = 1.0, 6.0, 11.0


def test_meta_persons_compact():
    tr = Tracker()
    for i in range(4):
        tr.update([ob(b, r=1 + b / 40) for b in (0, 40, 80, 120, 160, 200, 240)], [ob(1, src="T")], i * 0.1)
    p = tr.persons(0.3)
    assert len(p) == 5 and p[0]["src"] == "CT"
    blob = json.dumps({"t": 1762590000.1, "acc": [1.5, 1.3, 21.3], "tilt": 5.3, "persons": p}, separators=(",", ":"))
    assert len(blob) < 600                                # no keypoints on these fake detections


def test_meta_keypoints_percent_of_box_and_size():
    from spotter.tracker import keypoints_pct
    d = det(0.2, 0.1, 0.4, 0.9)
    d.kpts[0] = (0.3, 0.5, 0.9)                           # nose in the middle of the box
    d.kpts[5] = (0.2, 0.1, 1.0)                           # top-left corner
    d.kpts[6] = (0.4, 0.9, 1.0)                           # bottom-right corner: clamped to 99
    kp = keypoints_pct(d)
    assert len(kp) == 34 and kp[0:2] == [50, 50] and kp[10:12] == [0, 0] and kp[12:14] == [99, 99]
    assert kp[2:4] == [-1, -1]                            # not visible
    tr = Tracker()
    for i in range(4):
        tr.update([Obs("C", b, 1 + b / 40, "L", "U", 88, det(0.2, 0.1, 0.4, 0.9), 0.3)
                   for b in (0, 40, 80, 120, 160, 200)], [], i * 0.1)
    for t in tr.tracks:                                   # worst case: every keypoint visible, two digits
        t.det.kpts[:] = (0.31, 0.52, 1.0)
    p = tr.persons(0.3)
    assert sum("kp" in x for x in p) == 3
    blob = json.dumps({"t": 1762590000.1, "acc": [1.5, 1.3, 21.3], "tilt": 5.3, "persons": p,
                       "lvl": 2, "rssi": -80}, separators=(",", ":"))
    assert len(blob) < 900


def test_person_alert_fits_lora():
    from spotter.main import person_alert
    tr = Tracker()
    for i in range(3):
        tr.update([], [ob(123.4, r=3.55, src="T", ps="D")], i * 0.1)
    msg = person_alert(tr.tracks[0], None, cfg(), 0.2)
    assert msg["b"] == 123 and msg["r"] == 36 and msg["k"] == 1 and msg["s"] == "T" and msg["th"] == 1
    msg["rt"] = ["B2", "B1"]
    msg["q"] = -100
    assert len(encode_line(msg)) < 200


# --- IMU -------------------------------------------------------------------------------------------------------

def test_imu_decoding_and_tilt():
    assert to_signed6(0x15) == 21 and to_signed6(0x3F) == -1 and to_signed6(0x20) == -32
    assert to_signed6(0x41) is None                       # alert bit: skip
    assert tilt_deg(REF) == pytest.approx(0, abs=1e-6)
    assert tilt_deg((21.33, 0, 0), (0, 0, 21.33)) == pytest.approx(90)
    imu = Imu()
    assert imu.feed([0x01, 0x01, 0x15], 1.0) and not imu.feed([0x41, 0x01, 0x15], 1.1)
    s = imu.latest()
    assert s.az == pytest.approx(21) and s.tilt < 5


# --- calibration fit -------------------------------------------------------------------------------------------

def test_calib_fit_recovers_yaw_hfov_direction():
    from spotter import calib
    from spotter.locate import bearing as brg
    rng = np.random.default_rng(1)
    frames = []
    for i in range(120):
        u = [0.5, 0.15, 0.85][i // 40] + rng.normal(0, 0.01)
        b_true = brg(u, 62.0, 350.0, -1)                  # camera yaw 350, mirrored, 62 deg wide
        a, d = scan_with([(b_true - 4, b_true + 4, 2.0)], wall_m=5.0)
        frames.append({"cam": [[u - 0.05, 0.2, u + 0.05, 0.9, 0.9]], "th": [],
                       "ang": np.round(a * 10).astype(int).tolist(), "mm": np.round(d).astype(int).tolist()})
    pts = calib.pairs(frames)["C"]
    assert len(pts) == 120
    pts += [(0.5, 200.0)] * 20                            # outliers: someone else / the wall
    yaw, hfov, direction, rms, frac = calib.fit(pts)
    assert frac > 0.8 and direction == -1 and abs(((yaw - 350) + 180) % 360 - 180) < 1.5 and abs(hfov - 62) < 3 and rms < 1.5


def test_calib_set_config_and_summary(tmp_path):
    from spotter import calib
    from spotter.imu import ImuSample
    p = tmp_path / "config.json"
    p.write_text('{\n  "a": 1, "calib_record_s": 0, "calib_tag": ""\n}\n')
    calib.set_config(p, calib_tag="lying", calib_record_s=25)
    assert json.loads(p.read_text()) == {"a": 1, "calib_record_s": 25, "calib_tag": "lying"}
    calib.set_config(p, calib_record_s=0, extra=2)
    assert json.loads(p.read_text())["extra"] == 2 and p.read_text().startswith('{\n  "a": 1, "calib_record_s": 0,')
    rec = calib.Recorder(tmp_path)
    c = {**cfg(), "calib_record_s": 5, "calib_tag": "lying"}
    det = body(0.2, 0.5, 0.7, 0.7, 85)
    o = observe(det, "C", None, c)
    for i in range(3):
        rec.step(c, [o], [], (np.arange(10.0) * 36, np.full(10, 2000.0)), ImuSample(0, 1.0, 2.0, 21.0, 3))
    rec.f.close()                                     # normally closed when calib_record_s runs out
    s = calib.summary(rec.path)
    assert "lying" in s["file"] and s["frames"] == 3 and s["cam_seen"] == 3 and s["cam_ps"]["D"] == 3
    assert s["acc_mean"] == [1.0, 2.0, 21.0]
