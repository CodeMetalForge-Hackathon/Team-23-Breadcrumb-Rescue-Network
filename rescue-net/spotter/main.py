"""Spotter main loop: detect -> fuse with thermal -> locate in the LiDAR frame -> track -> rate-limit -> send.

python -m spotter.main [--mock] [--mock-thermal] [--out serial|stdout] [--port /dev/ttyUSB0] [--no-debugview]
"""
import argparse
import inspect
import logging
import sys
import time
from collections import deque

from common.protocol import make_msg
from spotter import calib, debugview, fusion, locate
from spotter.config import DEFAULT_PATH, Config
from spotter.debounce import Debouncer, RateLimiter
from spotter.imu import roll_deg
from spotter.tracker import Tracker

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
    th_det = cam = lid = None
    tracker = Tracker(merge_deg=cfg["track_merge_deg"])
    imu = None
    if not args.mock:
        from spotter.imu import Imu
        imu = Imu().start()
    shared = {"persons": []}                       # written by the loop, read by meta() on the bulk thread
    recorder = calib.Recorder()                    # camera -> LiDAR calibration data, see spotter/calib.py

    def meta():
        """Rig state for the map (bulk kind 4), < 600 B."""
        s = imu.latest() if imu else None
        return {"t": round(time.time(), 1),
                "acc": None if s is None else [round(s.ax, 1), round(s.ay, 1), round(s.az, 1)],
                "tilt": None if s is None or s.tilt is None else round(s.tilt, 1),
                "persons": shared["persons"]}

    if view is not None and not args.no_extra_sensors:
        cam, lid = add_extra_views(view, cfg, shared)
        if hasattr(detector, "infer") and cam.device:      # pose model on the thermal camera too
            from spotter import ircam
            from spotter.thermal_detect import ThermalDetector
            th_det = ThermalDetector(detector, cam)
            view.extras["thermal"] = lambda q: ircam.render(view.cv2, cam.latest(), cam.status, cam.fps,
                                                            dets=th_det.dets, cfg=cfg.data, labels=view.labels)
        if not args.no_bulk:
            from spotter.bulklink import BulkLink
            params = inspect.signature(BulkLink).parameters    # older bulklink.py: no meta / IMU
            extra = {k: v for k, v in (("meta_fn", meta), ("imu", imu)) if k in params}
            BulkLink(view, cam, lid, period=float(cfg.data.get("bulk_period", 0.25)),
                     gap_s=float(cfg.data.get("bulk_gap", 0.003)), **extra).start()
            if cfg.data.get("link_watch", True):    # B2 rebooting leaves a stale Wi-Fi association
                from spotter.linkwatch import LinkWatch
                LinkWatch(con=cfg.data.get("link_con", "RESCUE-LINK")).start()
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
            th_dets = th_det.step(cfg.data) if th_det else None
            analysis = fusion.analyze(dets, frame, cfg, [d for _, ds in recent for d in ds],
                                      th_dets=th_dets, th_x_scale=th_det.x_scale if th_det else 1.25)
            recent.append((now, dets))
            reject_log(analysis, now)

            # people: locate in the LiDAR frame, track, one alert stream per person
            scan = lid.latest() if lid else None
            people = {id(p.det): p for p in analysis.persons}
            imu_now = imu.latest() if imu else None
            roll = roll_deg(imu_now, cfg["imu_roll_axis"])         # 0 until the sideways axis is known
            cam_obs = [locate.observe(p.det, "C", scan, cfg.data, roll_deg=roll) for p in analysis.persons
                       if p.decision != "rejected"]
            th_obs = [locate.observe(d, "T", scan, cfg.data, x_scale=th_det.x_scale, roll_deg=roll)
                      for d in (th_dets or ())]
            recorder.step(cfg.data, cam_obs, th_obs, scan, imu_now)
            tracker.merge_deg = cfg["track_merge_deg"]
            tracker.cam_fov = (cfg["cam_yaw"], cfg["cam_hfov"] / 2 - 3)
            tracker.update(cam_obs, th_obs, now)
            shared["persons"] = tracker.persons(now)
            if view:
                view.labels = {id(t.det): f"#{t.id} [{t.ps or '?'}] {t.b:.0f}deg"
                               + ("" if t.r is None else f" {t.r:.1f}m{t.rs}")
                               for t in tracker.tracks if t.last_seen == now}
            for t in tracker.due(now, cfg["cooldown_s"]):
                msg = person_alert(t, people.get(id(t.det)), cfg, now)
                (outbox.appendleft if t.ps == "D" else outbox.append)(msg)    # someone down goes out first
                log.info("ALERT PERSON #%d b=%.0f r=%s src=%s ps=%s c=%d", t.id, t.b,
                         "-" if t.r is None else f"{t.r:.1f}m({t.rs})", t.src(now) or "-", t.ps or "-", t.c)

            best = {}                              # HEAT (MLX90640 only): strongest per view, debounced
            for cand in analysis.candidates:
                if cand["t"] == "PERSON":
                    continue
                key = (cand["t"], cand["v"])
                if key not in best or cand["c"] > best[key]["c"]:
                    best[key] = cand
            debouncer.cooldown_s = cfg["cooldown_s"]
            for key in debouncer.update(best, now):
                c = best[key]
                outbox.append(make_msg(c["t"], cfg["src"], cfg["ttl"], c=c["c"], z=cfg["zone"], v=c["v"], dt=c["dt"]))
                log.info("ALERT %s view=%s c=%d dt=%s", c["t"], c["v"], c["c"], c["dt"])

            if last_hb is None or now - last_hb >= cfg["hb_interval_s"]:
                outbox.append(make_msg("HB", cfg["src"], cfg["ttl"], z=cfg["zone"]))
                last_hb = now
            while len(outbox) > 20:                # drop the oldest, but never a "person down" alert
                i = next((k for k, m in enumerate(outbox) if m.get("ps") != "D"), 0)
                log.warning("outbox full, dropped %s", outbox[i])
                del outbox[i]
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


def person_alert(t, person, cfg, now):
    """PERSON LoRa alert for one track: b = bearing (deg, LiDAR frame), r = range (dm), k = track id."""
    src = t.src(now)
    dt = None if person is None or person.dt is None else round(person.dt)
    if "T" in src:
        th = 1                                         # the thermal camera sees this person
    elif dt is not None:
        th = 1 if person.decision == "confirmed" else 0   # MLX90640 heat check
    else:
        th = None
    return make_msg("PERSON", cfg["src"], cfg["ttl"], c=t.c, th=th, z=cfg["zone"], v=fusion.view_of(t.u), dt=dt,
                    ps=t.ps, s="T" if src == "T" else None, b=round(t.b),
                    r=None if t.r is None else round(t.r * 10), k=t.id)


def add_extra_views(view, cfg, shared):
    """USB thermal camera and RPLIDAR panes in the debug view; the LiDAR pane shows tracked people."""
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
        c = cfg.data
        fovs = [(c["cam_yaw"], c["cam_hfov"], (200, 200, 200)), (c["th_yaw"], c["th_hfov"], (60, 120, 255))]
        return lidar.render(cv2, lid.latest(), max_m=r, status=lid.status, hz=lid.scan_hz,
                            persons=shared["persons"], fovs=fovs)
    view.extras["lidar"] = lidar_view
    return cam, lid


if __name__ == "__main__":
    main()
