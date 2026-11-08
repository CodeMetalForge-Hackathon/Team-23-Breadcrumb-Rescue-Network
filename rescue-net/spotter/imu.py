"""Grove 3-axis accelerometer (MMA7660FC) on the Pi's I2C bus 1, address 0x4C: rig tilt for the map.

Registers: 0x00-0x02 = X, Y, Z as 6-bit two's complement (about 21.33 counts per g); bit 6 (0x40) = "alert",
the sample was being updated while read, so it is skipped. 0x07 = mode (0 standby, 1 active), 0x08 = sample rate.
latest() -> ImuSample(t, ax, ay, az, tilt): low-passed counts, and degrees between gravity now and gravity with
the rig level (REF, measured on a level table; the module itself sits 5.3 deg off level). None before the first sample.
"""
import logging
import math
import threading
import time
from collections import namedtuple

log = logging.getLogger("imu")

ADDR = 0x4C
COUNTS_PER_G = 21.33
REF = (1.45, 1.33, 21.26)
ImuSample = namedtuple("ImuSample", "t ax ay az tilt")


def to_signed6(b):
    """MMA7660 axis byte -> counts (-32..31), or None if the alert bit says the value is unreliable."""
    if b & 0x40:
        return None
    v = b & 0x3F
    return v - 64 if v & 0x20 else v


def tilt_deg(a, ref=REF):
    """Angle in degrees between acceleration vector a and the level reference."""
    na, nr = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in ref))
    if na < 1e-6 or nr < 1e-6:
        return None
    c = sum(x * y for x, y in zip(a, ref)) / (na * nr)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class Imu:
    def __init__(self, bus=1, addr=ADDR, hz=50.0, alpha=0.2, ref=REF):
        self.bus, self.addr, self.period, self.alpha, self.ref = bus, addr, 1.0 / hz, alpha, ref
        self._sample = None
        self._f = None                  # filtered (ax, ay, az)
        self.status = "starting"
        self._stop = False

    def start(self):
        threading.Thread(target=self._run, name="imu", daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    def latest(self):
        return self._sample             # a tuple, replaced whole: no lock needed

    def feed(self, raw, now):
        """One raw 3-byte read -> update the filter. Returns False if the sample was skipped."""
        vals = [to_signed6(b) for b in raw[:3]]
        if len(vals) < 3 or None in vals:
            return False
        if self._f is None:
            self._f = tuple(float(v) for v in vals)
        else:
            self._f = tuple(f + self.alpha * (v - f) for f, v in zip(self._f, vals))
        self._sample = ImuSample(now, *self._f, tilt_deg(self._f, self.ref))
        return True

    def _run(self):
        try:
            from smbus2 import SMBus
        except ImportError:
            self.status = "no smbus2"
            log.warning("imu: smbus2 not installed")
            return
        while not self._stop:
            try:
                with SMBus(self.bus) as bus:
                    bus.write_byte_data(self.addr, 0x07, 0x00)     # standby to change settings
                    bus.write_byte_data(self.addr, 0x08, 0x00)     # 120 samples/s
                    bus.write_byte_data(self.addr, 0x07, 0x01)     # active
                    self.status = "ok"
                    fails = 0
                    while not self._stop:
                        t0 = time.monotonic()
                        try:
                            self.feed(bus.read_i2c_block_data(self.addr, 0x00, 3), time.time())
                            fails = 0
                        except OSError:
                            fails += 1
                            if fails > 25:
                                raise
                        time.sleep(max(0.0, self.period - (time.monotonic() - t0)))
            except OSError as e:
                if self.status != f"error: {e}":
                    log.warning("imu: %s, retrying", e)
                self.status = f"error: {e}"
                time.sleep(2)


if __name__ == "__main__":                                # quick check: python3 -m spotter.imu
    logging.basicConfig(level=logging.INFO)
    imu = Imu().start()
    for _ in range(10):
        time.sleep(0.5)
        s = imu.latest()
        print(imu.status, None if s is None else f"ax {s.ax:+.2f} ay {s.ay:+.2f} az {s.az:+.2f} tilt {s.tilt:.1f} deg")
