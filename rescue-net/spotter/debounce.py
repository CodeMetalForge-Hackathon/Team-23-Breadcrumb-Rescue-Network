"""No tracking: an alert key (type, view) must be seen in 3 of the last 5 frames, then cools down."""
from collections import deque


class Debouncer:
    def __init__(self, window=5, need=3, cooldown_s=10.0):
        self.need = need
        self.cooldown_s = cooldown_s
        self.history = deque(maxlen=window)
        self.last_fired = {}

    def update(self, keys, now):
        """Feed the keys present in this frame. Returns the keys to alert on now."""
        keys = set(keys)
        self.history.append(keys)
        fire = []
        for key in sorted(keys):
            hits = sum(key in frame for frame in self.history)
            last = self.last_fired.get(key)
            if hits >= self.need and (last is None or now - last >= self.cooldown_s):
                self.last_fired[key] = now
                fire.append(key)
        return fire


class RateLimiter:
    """At most one event per min_interval_s."""

    def __init__(self, min_interval_s=1.0):
        self.min_interval_s = min_interval_s
        self.last = None

    def ready(self, now):
        return self.last is None or now - self.last >= self.min_interval_s

    def mark(self, now):
        self.last = now
