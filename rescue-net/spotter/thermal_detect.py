"""Person + pose detection on the USB thermal camera, so the spotter keeps working with the lights off.

The same Hailo pose model runs on the thermal frame (white-hot grey looks enough like a photo for a
COCO-trained model). The 640x512 frame is letterboxed into the 640x640 model input, and the detections
are mapped back to the thermal frame's own 0..1 coordinates.
"""
import logging
import time

import numpy as np

log = logging.getLogger("thermal_det")


def letterbox(img, input_hw):
    """Grey or BGR frame -> (model input HxWx3 uint8, (x0, y0, nw, nh) of the image inside it)."""
    import cv2
    h, w = input_hw
    ih, iw = img.shape[:2]
    s = min(w / iw, h / ih)
    nw, nh = int(round(iw * s)), int(round(ih * s))
    x0, y0 = (w - nw) // 2, (h - nh) // 2
    src = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img
    canvas = np.zeros((h, w, 3), np.uint8)
    canvas[y0:y0 + nh, x0:x0 + nw] = cv2.resize(src, (nw, nh), interpolation=cv2.INTER_AREA)
    return canvas, (x0, y0, nw, nh)


def unletterbox(det, box, input_hw):
    """Detection in model-input coordinates -> the same detection in the original frame's 0..1 coordinates."""
    h, w = input_hw
    x0, y0, nw, nh = box

    def fx(u):
        return (u * w - x0) / nw

    def fy(v):
        return (v * h - y0) / nh

    fields = dict(x0=fx(det.x0), y0=fy(det.y0), x1=fx(det.x1), y1=fy(det.y1))
    kpts = getattr(det, "kpts", None)
    if kpts is not None:
        k = np.array(kpts, dtype=np.float32, copy=True)
        k[:, 0], k[:, 1] = fx(k[:, 0]), fy(k[:, 1])
        fields["kpts"] = k
    return det._replace(**fields)


class ThermalDetector:
    """Runs the detector's model on the newest thermal frame every `thermal_det_every` RGB frames."""

    def __init__(self, detector, cam):
        self.detector, self.cam = detector, cam
        self.dets = []                 # last result, in thermal-frame coordinates
        self.frame_size = None         # (w, h) of the frame the result belongs to
        self.t = 0.0
        self._n = 0

    @property
    def x_scale(self):
        """Pixel aspect for posture(): normalised x * x_scale is comparable to normalised y."""
        return 1.25 if self.frame_size is None else self.frame_size[0] / self.frame_size[1]

    def step(self, cfg):
        """Call once per RGB frame. Returns the current thermal person detections (maybe from an earlier frame)."""
        if not cfg.get("thermal_detect", True):
            self.dets = []
            return self.dets
        self._n += 1
        if self._n % max(1, int(cfg.get("thermal_det_every", 2))):
            return self.dets if time.monotonic() - self.t < 1.0 else []
        frame = self.cam.latest()
        if frame is None:
            self.dets = []
            return self.dets
        canvas, box = letterbox(frame, self.detector.input_hw)
        try:
            raw = self.detector.infer(canvas)
        except Exception as e:                       # never let the thermal path stop the spotter
            log.warning("thermal inference failed: %s", e)
            self.dets = []
            return self.dets
        min_score = float(cfg.get("thermal_min_score", 0.45))
        self.dets = [unletterbox(d, box, self.detector.input_hw) for d in raw
                     if d.label == "person" and d.score >= min_score]
        self.frame_size = (frame.shape[1], frame.shape[0])
        self.t = time.monotonic()
        return self.dets
