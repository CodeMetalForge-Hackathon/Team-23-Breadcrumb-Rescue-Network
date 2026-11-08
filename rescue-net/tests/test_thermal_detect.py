import numpy as np
import pytest

from spotter.config import DEFAULTS
from spotter.detector import Detection, PoseDetection
from spotter.fusion import analyze
from spotter.thermal_detect import ThermalDetector, letterbox, unletterbox


def cfg(**over):
    c = dict(DEFAULTS)
    c.update(over)
    return c


def pose_det(x0, y0, x1, y1, score=0.8, lying=False):
    """Pose detection with shoulders/hips placed for standing or lying."""
    k = np.zeros((17, 3), np.float32)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if lying:      # torso horizontal
        k[5], k[6], k[11], k[12] = (x0, cy, 1), (x0, cy, 1), (x1, cy, 1), (x1, cy, 1)
    else:          # torso vertical
        k[5], k[6], k[11], k[12] = (cx, y0, 1), (cx, y0, 1), (cx, y1, 1), (cx, y1, 1)
    return PoseDetection("person", score, x0, y0, x1, y1, k)


def test_letterbox_round_trip():
    pytest.importorskip("cv2")
    frame = np.zeros((512, 640), np.uint8)
    canvas, box = letterbox(frame, (640, 640))
    assert canvas.shape == (640, 640, 3)
    assert box == (0, 64, 640, 512)                  # 640x512 -> 64 px bars top and bottom
    # a box covering the thermal frame's middle, expressed in model-input coordinates
    d = pose_det(0.25, (64 + 0.25 * 512) / 640, 0.75, (64 + 0.75 * 512) / 640)
    back = unletterbox(d, box, (640, 640))
    assert back.x0 == pytest.approx(0.25) and back.y0 == pytest.approx(0.25)
    assert back.x1 == pytest.approx(0.75) and back.y1 == pytest.approx(0.75)
    assert back.kpts[5][1] == pytest.approx(0.25, abs=1e-5)   # keypoints mapped too


def test_unletterbox_plain_detection_without_keypoints():
    d = Detection("person", 0.9, 0.0, 0.1, 1.0, 0.9)
    back = unletterbox(d, (0, 64, 640, 512), (640, 640))
    assert back.y0 == pytest.approx((0.1 * 640 - 64) / 512)


def test_thermal_only_person_when_camera_sees_nobody():
    th = [pose_det(0.1, 0.6, 0.6, 0.8, lying=True)]
    a = analyze([], None, cfg(), th_dets=th, th_x_scale=1.25)
    [c] = a.candidates
    assert c["t"] == "PERSON" and c["s"] == "T" and c["th"] == 1 and c["ps"] == "D"
    assert a.th_persons == tuple(th)


def test_camera_person_confirmed_by_thermal_camera():
    rgb = [pose_det(0.4, 0.2, 0.6, 0.9)]
    a = analyze(rgb, None, cfg(), th_dets=[pose_det(0.3, 0.2, 0.5, 0.9)])
    [c] = a.candidates                                # no duplicate thermal-only alert
    assert c["th"] == 1 and "s" not in c and c["ps"] == "U"


def test_without_thermal_camera_nothing_changes():
    a = analyze([pose_det(0.4, 0.2, 0.6, 0.9)], None, cfg())
    [c] = a.candidates
    assert c["th"] is None and "s" not in c and a.th_persons == ()


class FakeCam:
    def __init__(self, frame):
        self.frame = frame

    def latest(self):
        return self.frame


class FakeDetector:
    input_hw = (640, 640)

    def __init__(self):
        self.calls = 0

    def infer(self, img):
        self.calls += 1
        assert img.shape == (640, 640, 3)
        return [pose_det(0.2, 0.3, 0.4, 0.7, score=0.9), pose_det(0.5, 0.3, 0.6, 0.7, score=0.2)]


def test_thermal_detector_runs_every_other_frame_and_filters_score():
    pytest.importorskip("cv2")
    det = FakeDetector()
    td = ThermalDetector(det, FakeCam(np.zeros((512, 640), np.uint8)))
    c = cfg(thermal_det_every=2, thermal_min_score=0.45)
    results = [td.step(c) for _ in range(4)]
    assert det.calls == 2
    assert all(len(r) == 1 for r in results[1:])    # the low-score one is dropped, result kept between runs
    assert td.x_scale == pytest.approx(1.25)


def test_thermal_detector_off_or_no_frame():
    det = FakeDetector()
    assert ThermalDetector(det, FakeCam(None)).step(cfg(thermal_det_every=1)) == []
    assert ThermalDetector(det, FakeCam(np.zeros((512, 640), np.uint8))).step(cfg(thermal_detect=False)) == []
    assert det.calls == 0
