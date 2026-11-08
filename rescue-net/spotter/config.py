"""Spotter tunables: defaults + config.json, re-read whenever the file changes."""
import json
import logging
import time
from pathlib import Path

log = logging.getLogger("config")

DEFAULT_PATH = Path(__file__).resolve().parent / "config.json"

DEFAULTS = {
    "src": "SP1", "zone": "B2", "ttl": 4,
    "model_path": "", "det_min_score": 0.5,
    "flip_h": False, "flip_v": False, "scale_x": 1.0, "scale_y": 1.0, "offset_x": 0.0, "offset_y": 0.0,
    "person_min_dt": 5.0, "reject_max_dt": 2.0,
    "heat_min_dt": 5.0, "heat_min_pixels": 3, "heat_conf": 40,
    "cooldown_s": 10, "hb_interval_s": 15,
    "serial_port": "", "baud": 115200, "debug_port": 8080,
    "down_angle_deg": 55, "down_aspect": 1.3,      # posture: torso this far from vertical (or box this wide) = on the ground
    "bulk_period": 0.25, "bulk_gap": 0.003,        # live view over B2 Wi-Fi: seconds per round, seconds between chunks
    "thermal_detect": True, "thermal_det_every": 2, "thermal_min_score": 0.45,   # pose model on the USB thermal camera
    # where people are, in the LiDAR frame: camera field of view (deg), yaw (LiDAR angle of the optical axis),
    # direction (+1: image right = clockwise), LiDAR window around a bearing, and the rig's own returns to ignore
    "cam_hfov": 66.0, "cam_yaw": 0.0, "cam_dir": 1, "th_hfov": 50.0, "th_yaw": 0.0, "th_dir": 1,
    "lidar_win_deg": 3.0, "lidar_min_m": 0.25, "track_merge_deg": 15.0,
    "calib_record_s": 0,                           # > 0: record that many seconds of calibration data to ~/logs
}


class Config:
    def __init__(self, path=DEFAULT_PATH):
        self.path = Path(path)
        self.data = dict(DEFAULTS)
        self._mtime = None
        self._checked = 0.0
        self.maybe_reload(force=True)

    def __getitem__(self, key):
        return self.data[key]

    def maybe_reload(self, force=False):
        """Cheap to call every frame: stats the file at most twice a second."""
        now = time.monotonic()
        if not force and now - self._checked < 0.5:
            return
        self._checked = now
        try:
            mtime = self.path.stat().st_mtime
        except FileNotFoundError:
            if force:
                log.warning("%s not found, using defaults", self.path)
            return
        if mtime == self._mtime:
            return
        self._mtime = mtime
        try:
            loaded = json.loads(self.path.read_text())
        except ValueError as e:
            log.error("config.json is invalid, keeping previous values: %s", e)
            return
        unknown = sorted(set(loaded) - set(DEFAULTS))
        if unknown:
            log.warning("unknown config keys ignored: %s", ", ".join(unknown))
        new = {**DEFAULTS, **{k: v for k, v in loaded.items() if k in DEFAULTS}}
        changed = {k: v for k, v in new.items() if self.data.get(k) != v}
        self.data = new
        if changed and not force:
            log.info("config reloaded: %s", changed)
