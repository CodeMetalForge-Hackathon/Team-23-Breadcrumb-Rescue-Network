"""MJPEG debug stream on http://<pi>:8080/ : camera + person boxes + thermal overlay + config.

Frames are only rendered while someone is watching, in the HTTP thread, so the detection loop
just hands over references and never waits on drawing or the network.
"""
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import numpy as np

log = logging.getLogger("debugview")

W, H = 640, 480
COLORS = {"confirmed": (0, 220, 0), "uncertain": (0, 220, 255), "rejected": (0, 0, 255)}   # BGR
HEAT_COLOR = (0, 140, 255)
PAGE = b"""<!doctype html><title>Spotter debug</title>
<body style="margin:0;background:#111;color:#ddd;font:14px sans-serif">
<img id="v" src="/stream" style="width:100%;max-width:960px;display:block;margin:auto">
<p style="text-align:center">green = person + heat &middot; yellow = person, heat unsure &middot;
red = rejected (no heat) &middot; orange = HEAT blob. Edit spotter/config.json to move the overlay.
<button onclick="var v=document.getElementById('v');v.src=v.src.indexOf('overlay=0')<0?'/stream?overlay=0':'/stream'">
toggle thermal overlay</button></p>"""


class DebugView:
    def __init__(self, port):
        import cv2                      # imported here so the spotter still runs without OpenCV
        self.cv2 = cv2
        self._lock = threading.Lock()
        self._state = None
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                url = urlsplit(self.path)
                overlay = "overlay=0" not in url.query
                if url.path == "/":
                    self._send(200, "text/html", PAGE)
                elif url.path == "/frame.jpg":
                    self._send(200, "image/jpeg", view.render_jpeg(overlay))
                elif url.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    try:
                        while True:
                            jpg = view.render_jpeg(overlay)
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                            time.sleep(0.1)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self._send(404, "text/plain", b"not found")

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, name="debugview", daemon=True).start()
        log.info("debug view on http://0.0.0.0:%d/", port)

    def update(self, image, analysis, thermal, cfg, fps):
        """Called from the detection loop: store references only."""
        with self._lock:
            self._state = (image, analysis, thermal, dict(cfg.data), fps)

    def render_jpeg(self, overlay=True):
        with self._lock:
            state = self._state
        img = self._render(*state, overlay=overlay) if state else self._blank("waiting for first frame")
        return self.cv2.imencode(".jpg", img, [self.cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()

    def _blank(self, text):
        img = np.full((H, W, 3), 40, np.uint8)
        self.cv2.putText(img, text, (W - 300, 24), self.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1)
        return img

    def _render(self, image, analysis, thermal, cfg, fps, overlay=True):
        cv2 = self.cv2
        img = image.copy() if image is not None else self._blank("MOCK CAMERA (no image)")
        h, w = img.shape[:2]
        if thermal is not None and overlay:
            self._overlay_thermal(img, thermal, cfg)

        for p in analysis.persons:
            d, color = p.det, COLORS[p.decision]
            x0, y0, x1, y1 = int(d.x0 * w), int(d.y0 * h), int(d.x1 * w), int(d.y1 * h)
            cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
            dt = "dt ?" if p.dt is None else f"dt{p.dt:+.1f}"
            label = f"{d.score:.2f} {dt}" + (" REJECTED" if p.decision == "rejected" else "")
            _text(cv2, img, label, (x0 + 2, max(y0 - 6, 14)), color)

        for b in analysis.blobs:
            center = (int(min(max(b.u, 0), 1) * w), int(min(max(b.v, 0), 1) * h))
            cv2.circle(img, center, 18, HEAT_COLOR, 2)
            _text(cv2, img, f"HEAT {b.dt:+.1f}", (center[0] - 30, center[1] - 24), HEAT_COLOR)

        amb = "none" if analysis.ambient is None else f"{analysis.ambient:.1f}C"
        lines = [
            f"fps {fps:.1f}   thermal {'ok' if thermal is not None else 'NONE'}   ambient {amb}",
            f"flip_h {cfg['flip_h']}  flip_v {cfg['flip_v']}",
            f"scale {cfg['scale_x']:.2f},{cfg['scale_y']:.2f}  offset {cfg['offset_x']:+.2f},{cfg['offset_y']:+.2f}",
            f"person>={cfg['person_min_dt']}  reject<{cfg['reject_max_dt']}  heat>={cfg['heat_min_dt']}",
        ]
        top = h - 10 - 18 * len(lines)               # bottom-left, so it never hides box labels
        cv2.rectangle(img, (0, top), (370, h), (0, 0, 0), -1)
        for i, line in enumerate(lines):
            cv2.putText(img, line, (6, top + 14 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    def _overlay_thermal(self, img, thermal, cfg, alpha=0.45):
        """Blend the thermal frame where the current mapping says it lands on the camera image."""
        cv2 = self.cv2
        h, w = img.shape[:2]
        t = thermal[:, ::-1] if cfg["flip_h"] else thermal       # undo flips: now t[0,0] is top-left in camera terms
        t = t[::-1, :] if cfg["flip_v"] else t
        amb = float(np.median(thermal))
        norm = np.clip((t - (amb - 2.0)) / 14.0 * 255, 0, 255).astype(np.uint8)
        color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # thermal edges (tu = 0 and 1, before flipping) mapped back to camera pixels
        x0 = ((0 - 0.5 - cfg["offset_x"]) / cfg["scale_x"] + 0.5) * w
        x1 = ((1 - 0.5 - cfg["offset_x"]) / cfg["scale_x"] + 0.5) * w
        y0 = ((0 - 0.5 - cfg["offset_y"]) / cfg["scale_y"] + 0.5) * h
        y1 = ((1 - 0.5 - cfg["offset_y"]) / cfg["scale_y"] + 0.5) * h
        tw, th = int(round(x1 - x0)), int(round(y1 - y0))
        if tw <= 0 or th <= 0:
            return
        big = cv2.resize(color, (tw, th), interpolation=cv2.INTER_CUBIC)
        X0, Y0 = int(round(x0)), int(round(y0))
        dx0, dy0, dx1, dy1 = max(X0, 0), max(Y0, 0), min(X0 + tw, w), min(Y0 + th, h)
        if dx1 <= dx0 or dy1 <= dy0:
            return
        src = big[dy0 - Y0:dy1 - Y0, dx0 - X0:dx1 - X0]
        img[dy0:dy1, dx0:dx1] = cv2.addWeighted(img[dy0:dy1, dx0:dx1], 1 - alpha, src, alpha, 0)


def _text(cv2, img, text, org, color):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def start(port):
    """DebugView, or None (with a warning) if OpenCV is missing or the port is taken."""
    try:
        return DebugView(port)
    except ImportError:
        log.warning("OpenCV not installed, debug view disabled")
    except OSError as e:
        log.warning("debug view disabled, port %d unavailable: %s", port, e)
    return None
