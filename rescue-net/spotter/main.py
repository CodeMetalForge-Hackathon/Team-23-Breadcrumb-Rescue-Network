"""Spotter main loop: detect -> fuse with thermal -> debounce -> rate-limit -> send.

python -m spotter.main [--mock] [--mock-thermal] [--out serial|stdout] [--port /dev/ttyUSB0] [--no-debugview]
"""
import argparse
import logging
import sys
import time
from collections import deque

from common.protocol import make_msg
from spotter import debugview, fusion
from spotter.config import DEFAULT_PATH, Config
from spotter.debounce import Debouncer, RateLimiter

log = logging.getLogger("spotter")


def parse_args():
    p = argparse.ArgumentParser(description="Rescue Net spotter")
    p.add_argument("--mock", action="store_true", help="fake camera + fake thermal (runs on a laptop)")
    p.add_argument("--mock-thermal", action="store_true", help="real camera + fake thermal")
    p.add_argument("--no-thermal", action="store_true", help="no thermal camera (this build): camera + pose only")
    p.add_argument("--out", choices=["serial", "stdout"], default="serial")
    p.add_argument("--port", help="serial port of the ESP32 (default: config serial_port, else auto-detect)")
    p.add_argument("--no-debugview", action="store_true")
    p.add_argument("--no-extra-sensors", action="store_true", help="do not show the USB thermal camera / LiDAR in the debug view")
    p.add_argument("--no-bulk", action="store_true", help="do not send camera / thermal / LiDAR over the bulk link (B2 Wi-Fi)")
    p.add_argument("--config", default=str(DEFAULT_PATH))
    p.add_argument("-v", "--verbose", action="store_true", help="debug logging (incl. lines the ESP32 prints)")
    return p.parse_args()


def build_sources(args, cfg):
    from spotter.detector import HailoDetector, MockDetector
    from spotter.thermal import MLX90640Source, MockThermal, NoThermal
    detector = MockDetector() if args.mock else HailoDetector(cfg)
    if args.no_thermal:
        thermal = NoThermal()
    else:
        thermal = MockThermal(detector) if (args.mock or args.mock_thermal) else MLX90640Source()
    return detector, thermal


def build_sink(args, cfg):
    from spotter.sink import SerialSink, StdoutSink
    if args.out == "stdout":
        return StdoutSink()
    return SerialSink(args.port or cfg["serial_port"] or None, cfg["baud"])


class RejectLog:
    """'REJECTED (no heat)' at most once per 2 s per view, so 30 FPS doesn't flood the log."""

    def __init__(self):
        self.last = {}

    def __call__(self, analysis, now):
        for p in analysis.persons:
            if p.decision != "rejected":
                continue
            view = fusion.view_of((p.det.x0 + p.det.x1) / 2)
            if now - self.last.get(view, -1e9) >= 2.0:
                self.last[view] = now
                log.info("REJECTED (no heat) view=%s score=%.2f dt=%+.1f", view, p.det.score, p.dt)


def main():
    args = parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, stream=sys.stderr,
                        format="%(asctime)s %(name)-9s %(message)s", datefmt="%H:%M:%S")
    cfg = Config(args.config)
    detector, thermal = build_sources(args, cfg)
    sink = build_sink(args, cfg)
    view = None if args.no_debugview else debugview.start(cfg["debug_port"])
    if view is not None and not args.no_extra_sensors:
        cam, lid = add_extra_views(view)
        if not args.no_bulk:
            from spotter.bulklink import BulkLink
            BulkLink(view, cam, lid).start()
    thermal.start()

    debouncer = Debouncer(cooldown_s=cfg["cooldown_s"])
    limiter = RateLimiter(1.0)
    outbox = deque()
    reject_log = RejectLog()
    recent = deque()                               # (time, person boxes) for the last second
    last_hb = None
    frames, fps, fps_t0 = 0, 0.0, time.monotonic()
    log.info("spotter %s in zone %s running (%s)", cfg["src"], cfg["zone"], "mock" if args.mock else "camera")

    try:
        while True:
            cfg.maybe_reload()
            image, dets = detector.detect()
            now = time.monotonic()
            dets = [d for d in dets if d.label == "person" and d.score >= cfg["det_min_score"]]
            frame = thermal.latest()
            while recent and now - recent[0][0] > 1.0:
                recent.popleft()
            analysis = fusion.analyze(dets, frame, cfg, [d for _, ds in recent for d in ds])
            recent.append((now, dets))
            reject_log(analysis, now)

            best = {}                              # strongest candidate per (type, view) this frame
            for cand in analysis.candidates:
                key = (cand["t"], cand["v"], cand.get("ps", ""))   # going down fires at once, not after the cooldown
                if key not in best or cand["c"] > best[key]["c"]:
                    best[key] = cand
            debouncer.cooldown_s = cfg["cooldown_s"]
            for key in debouncer.update(best, now):
                c = best[key]
                outbox.append(make_msg(c["t"], cfg["src"], cfg["ttl"], c=c["c"], th=c.get("th"),
                                       z=cfg["zone"], v=c["v"], dt=c["dt"], ps=c.get("ps") or None))
                log.info("ALERT %s view=%s c=%d th=%s dt=%s ps=%s", c["t"], c["v"], c["c"], c.get("th", "-"),
                         c["dt"], c.get("ps") or "-")

            if last_hb is None or now - last_hb >= cfg["hb_interval_s"]:
                outbox.append(make_msg("HB", cfg["src"], cfg["ttl"], z=cfg["zone"]))
                last_hb = now
            while len(outbox) > 20:
                log.warning("outbox full, dropped %s", outbox.popleft())
            if outbox and limiter.ready(now):
                try:
                    sink.send(outbox.popleft())
                except ValueError:
                    pass                           # oversized; already logged by encode_line
                limiter.mark(now)

            frames += 1
            if now - fps_t0 >= 2.0:
                fps, frames, fps_t0 = frames / (now - fps_t0), 0, now
            if view:
                view.update(image, analysis, frame, cfg, fps)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        thermal.stop()
        sink.close()
        log.info("spotter stopped")


def add_extra_views(view):
    """USB thermal camera and RPLIDAR panes in the debug view (display only for now)."""
    from spotter import ircam, lidar
    cv2 = view.cv2
    cam = ircam.IRCamera().start()
    lid = lidar.Lidar().start()
    view.extras["thermal"] = lambda q: ircam.render(cv2, cam.latest(), cam.status, cam.fps)

    def lidar_view(q):
        try:
            r = min(max(float(q.get("r", 6)), 1.0), 12.0)
        except ValueError:
            r = 6.0
        return lidar.render(cv2, lid.latest(), max_m=r, status=lid.status, hz=lid.scan_hz)
    view.extras["lidar"] = lidar_view
    return cam, lid


if __name__ == "__main__":
    main()
