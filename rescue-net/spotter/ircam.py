"""USB (UVC) thermal camera, e.g. the 640x512 "IRcamera" that shows up as /dev/videoN.

It outputs an 8-bit white-hot grey image (AGC on the camera, no absolute temperature).
latest() returns the last frame as a 2-D uint8 array, or None.
"""
import glob
import logging
import threading
import time

log = logging.getLogger("ircam")


def find_device(name_hint="IRcamera"):
    """Lowest /dev/videoN whose V4L2 name contains name_hint (the capture node, not the metadata one)."""
    hits = []
    for path in glob.glob("/sys/class/video4linux/video*"):
        try:
            if name_hint.lower() in open(path + "/name").read().lower():
                hits.append(int(path.rsplit("video", 1)[1]))
        except (OSError, ValueError):
            pass
    return f"/dev/video{min(hits)}" if hits else None


class IRCamera:
    def __init__(self, device=None, width=640, height=512):
        self.device, self.size = device or find_device(), (width, height)
        self._lock = threading.Lock()
        self._frame = None
        self.fps = 0.0
        self.status = "starting"
        self._stop = False

    def start(self):
        if not self.device:
            self.status = "no thermal camera"
            log.warning("ircam: no UVC thermal camera found")
            return self
        threading.Thread(target=self._run, name="ircam", daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    def latest(self):
        with self._lock:
            return self._frame

    def _run(self):
        import cv2
        while not self._stop:
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            if not cap.isOpened():
                self.status = "cannot open " + self.device
                time.sleep(2)
                continue
            self.status, t_last, fails = "ok", time.time(), 0
            while not self._stop and fails < 50:
                ok, img = cap.read()
                if not ok:
                    fails += 1
                    time.sleep(0.05)
                    continue
                fails = 0
                grey = img[:, :, 0] if img.ndim == 3 else img     # YUYV grey -> all channels equal
                now = time.time()
                self.fps = 0.9 * self.fps + 0.1 / max(now - t_last, 1e-3)
                t_last = now
                with self._lock:
                    self._frame = grey
            cap.release()
            self.status = "lost, reopening"
            time.sleep(1)


def render(cv2, frame, status="", fps=0.0):
    """False-colour view (inferno: hot = bright yellow)."""
    if frame is None:
        img = __import__("numpy").full((512, 640, 3), 30, "uint8")
        cv2.putText(img, f"THERMAL {status}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 1)
        return img
    img = cv2.applyColorMap(frame, cv2.COLORMAP_INFERNO)
    cv2.rectangle(img, (0, img.shape[0] - 26), (420, img.shape[0]), (0, 0, 0), -1)
    cv2.putText(img, f"THERMAL {status}  {fps:.0f} fps  (relative, white-hot AGC)",
                (6, img.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (230, 230, 230), 1)
    return img
