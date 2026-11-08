"""Person detectors. detect() returns (image_or_None, [Detection]) with normalized 0..1 coords."""
import math
import random
import time
from collections import namedtuple

import numpy as np

Detection = namedtuple("Detection", "label score x0 y0 x1 y1")
PoseDetection = namedtuple("PoseDetection", "label score x0 y0 x1 y1 kpts")   # kpts: 17 x (x, y, conf), COCO order


def _clamp_box(x0, y0, x1, y1):
    return max(0.0, x0), max(0.0, y0), min(1.0, x1), min(1.0, y1)


class MockDetector:
    """Fake camera at ~10 FPS: one person drifting left<->right (absent now and then),
    plus a 'poster' of a person on the left wall that has no body heat (must be rejected)."""

    def __init__(self, fps=10.0):
        self.period = 1.0 / fps
        self.t0 = time.monotonic()
        self._next = self.t0
        self.person = None      # true person box, read by MockThermal to place the body heat
        self.poster = None

    def detect(self):
        self._next += self.period
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()
        t = time.monotonic() - self.t0

        dets = []
        if t % 32 < 24:                                   # person visible 24 s out of every 32 s
            u = 0.5 + 0.38 * math.sin(2 * math.pi * t / 40)
            self.person = _clamp_box(u - 0.09, 0.25, u + 0.09, 0.85)
            if random.random() < 0.9:                     # the detector misses some frames
                dets.append(Detection("person", random.uniform(0.70, 0.95), *self.person))
        else:
            self.person = None

        if 20 <= t % 45 < 32:                             # poster visible 12 s out of every 45 s
            self.poster = (0.04, 0.10, 0.20, 0.55)
            dets.append(Detection("person", random.uniform(0.60, 0.80), *self.poster))
        else:
            self.poster = None
        return None, dets


MODEL_DIR = "/usr/share/hailo-models"
# COCO detectors, best first. *_h8 is compiled for Hailo-8; *_h8l (Hailo-8L) also runs on Hailo-8.
PREFERRED_MODELS = ["yolov8s_h8.hef", "yolov6n_h8.hef", "yolov8s_h8l.hef", "yolov6n_h8l.hef",
                    "yolox_s_leaky_h8l_rpi.hef"]


def pick_model():
    import os
    available = set(os.listdir(MODEL_DIR)) if os.path.isdir(MODEL_DIR) else set()
    for name in PREFERRED_MODELS:
        if name in available:
            return os.path.join(MODEL_DIR, name)
    raise FileNotFoundError(f"no known COCO detector in {MODEL_DIR} (found: {sorted(available)}); "
                            "set model_path in config.json")


def parse_nms(output, person_class=0):
    """Hailo NMS output -> person Detections. Expected (as in picamera2's Hailo example):
    a per-class list, each an array of [y0, x0, y1, x1, score] rows in normalized coords."""
    if isinstance(output, dict):                       # several output layers: take the NMS one
        output = next(iter(output.values()))
    if len(output) == 1 and len(output[0]) > person_class and hasattr(output[0][0], "__len__"):
        output = output[0]                             # leading batch dimension
    return [Detection("person", float(d[4]), float(d[1]), float(d[0]), float(d[3]), float(d[2]))
            for d in output[person_class]]


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _nms(boxes, scores, iou_thr):
    order, keep = np.argsort(-scores), []
    while order.size:
        i = order[0]
        keep.append(i)
        xx0 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy0 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx1 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy1 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.clip(xx1 - xx0, 0, None) * np.clip(yy1 - yy0, 0, None)
        area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
        iou = inter / (area(boxes[i]) + area(boxes[order[1:]]) - inter + 1e-9)
        order = order[1:][iou < iou_thr]
    return keep


def decode_yolov8_pose(outputs, input_hw, score_thr=0.25, iou_thr=0.6, reg_max=16):
    """Raw YOLOv8-pose heads (per stride: 64-ch DFL box, 1-ch person score, 51-ch keypoints) -> [PoseDetection].
    Hailo's yolov8s_pose_h8.hef ends at these convs (no on-chip NMS); decode as in Ultralytics."""
    H, W = input_hw
    groups = {}
    for arr in outputs.values():
        a = np.asarray(arr, dtype=np.float32)
        if a.ndim == 4:
            a = a[0]
        groups.setdefault(a.shape[:2], {})[a.shape[2]] = a
    boxes, scores, kpts = [], [], []
    for (h, w), g in groups.items():
        if not {64, 1, 51} <= set(g):
            continue
        stride = H / h
        s = g[1].reshape(-1)
        if s.min() < 0 or s.max() > 1:                  # logits, not probabilities
            s = _sigmoid(s)
        idx = np.nonzero(s >= score_thr)[0]
        if not idx.size:
            continue
        gy, gx = (idx // w).astype(np.float32), (idx % w).astype(np.float32)
        b = g[64].reshape(-1, 4, reg_max)[idx]
        e = np.exp(b - b.max(-1, keepdims=True))
        d = (e / e.sum(-1, keepdims=True) * np.arange(reg_max)).sum(-1)       # l, t, r, b in grid cells
        cx, cy = gx + 0.5, gy + 0.5
        boxes.append(np.stack([(cx - d[:, 0]) * stride / W, (cy - d[:, 1]) * stride / H,
                               (cx + d[:, 2]) * stride / W, (cy + d[:, 3]) * stride / H], axis=1))
        k = g[51].reshape(-1, 17, 3)[idx]
        kpts.append(np.stack([(k[..., 0] * 2 + gx[:, None]) * stride / W,
                              (k[..., 1] * 2 + gy[:, None]) * stride / H, _sigmoid(k[..., 2])], axis=-1))
        scores.append(s[idx])
    if not boxes:
        return []
    boxes, scores, kpts = np.concatenate(boxes), np.concatenate(scores), np.concatenate(kpts)
    out = []
    for i in _nms(boxes, scores, iou_thr):
        x0, y0, x1, y1 = (float(v) for v in np.clip(boxes[i], 0.0, 1.0))
        out.append(PoseDetection("person", float(scores[i]), x0, y0, x1, y1, kpts[i]))
    return out


class HailoDetector:
    """Pi camera + Hailo-8 via picamera2's Hailo helper. The lores stream is sized to the model input
    and fed to the Hailo; the main stream (same field of view) is kept for the debug view."""

    def __init__(self, cfg, main_size=(1280, 720)):
        import logging
        from picamera2 import Picamera2
        from picamera2.devices import Hailo
        path = cfg["model_path"] or pick_model()
        self.hailo = Hailo(path)
        h, w, _ = self.hailo.get_input_shape()
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": main_size, "format": "RGB888"},       # BGR byte order: ready for OpenCV
            lores={"size": (w, h), "format": "RGB888"},
            controls={"FrameRate": 30}))
        if "AfMode" in self.picam2.camera_controls:         # Camera Module 3: keep people in focus
            from libcamera import controls
            self.picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        self.picam2.start()
        self.input_hw = (h, w)
        self.person = None
        logging.getLogger("detector").info("Hailo model %s, input %dx%d", path, w, h)

    def detect(self):
        request = self.picam2.capture_request()          # main + lores from the same exposure
        try:
            image = request.make_array("main")
            lores = request.make_array("lores")
        finally:
            request.release()
        out = self.hailo.run(lores)
        if isinstance(out, dict) and len(out) > 1:          # pose model: raw heads, decode on the CPU
            dets = decode_yolov8_pose(out, self.input_hw)
        else:
            dets = parse_nms(out)
        best = max(dets, key=lambda d: d.score, default=None)
        self.person = None if best is None else (best.x0, best.y0, best.x1, best.y1)   # for --mock-thermal
        return image, dets
