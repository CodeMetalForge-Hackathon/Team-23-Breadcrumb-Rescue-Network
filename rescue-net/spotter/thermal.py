"""Thermal sources. A reader thread keeps only the latest (24, 32) °C frame; latest() never blocks."""
import logging
import threading
import time

import numpy as np

log = logging.getLogger("thermal")

ROWS, COLS = 24, 32
MAX_AGE_S = 2.0


class ThermalSource:
    def __init__(self, rate_hz=4.0):
        self.period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._run, name=type(self).__name__, daemon=True).start()
        return self

    def stop(self):
        self._stop.set()

    def latest(self):
        """Newest frame, or None if there is none or it is older than 2 s."""
        with self._lock:
            if self._frame is None or time.monotonic() - self._stamp > MAX_AGE_S:
                return None
            return self._frame

    def _publish(self, frame):
        with self._lock:
            self._frame = frame
            self._stamp = time.monotonic()

    def _run(self):
        raise NotImplementedError


class NoThermal:
    """No thermal camera on this build: latest() is always None, so PERSON alerts carry no th/dt."""

    def start(self):
        return self

    def stop(self):
        pass

    def latest(self):
        return None


class MockThermal(ThermalSource):
    """Room at ~22 °C, a ~31 °C body that follows MockDetector's true person box
    (with a lukewarm phase to exercise th=0), and a radiator that comes and goes (HEAT)."""

    def __init__(self, detector, rate_hz=4.0):
        super().__init__(rate_hz)
        self.detector = detector
        self.rng = np.random.default_rng()

    def _paint_box(self, frame, box, temp):
        x0, y0, x1, y1 = box
        w, h = x1 - x0, y1 - y0       # body heat fills the middle of the box (assumes calibrated mapping)
        c0, c1 = int((x0 + 0.2 * w) * COLS), int(np.ceil((x1 - 0.2 * w) * COLS))
        r0, r1 = int((y0 + 0.1 * h) * ROWS), int(np.ceil((y1 - 0.2 * h) * ROWS))
        frame[r0:r1, c0:c1] = temp + self.rng.normal(0, 0.5, (r1 - r0, c1 - c0))

    def _run(self):
        t0 = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic() - t0
            frame = 22.0 + self.rng.normal(0, 0.3, (ROWS, COLS))
            person = self.detector.person
            if person is not None:
                lukewarm = 40 <= t % 60 < 50          # thick jacket: only +3.5 °C -> th=0
                self._paint_box(frame, person, 25.5 if lukewarm else 31.0)
            if t % 30 < 12:                           # radiator bottom-right, no person -> HEAT
                frame[18:21, 27:30] = 30.0 + self.rng.normal(0, 0.3, (3, 3))
            self._publish(frame)
            time.sleep(self.period)


class MLX90640Source(ThermalSource):
    """Adafruit MLX90640 at 0x33. Written after the M0 envcheck confirms the sensor."""

    def __init__(self, rate_hz=4.0):
        raise NotImplementedError("MLX90640Source is not written yet (waiting on M0 envcheck). Use --mock-thermal.")
