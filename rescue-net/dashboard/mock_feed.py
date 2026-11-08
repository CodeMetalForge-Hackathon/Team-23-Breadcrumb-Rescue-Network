"""Realistic fake traffic as the base ESP32 would print it, for UI work without radios.

python -m dashboard.mock_feed [--speed 5]      # prints JSON lines to stdout in real time
"""
import argparse
import json
import random
import time

from common.protocol import encode_line, make_msg

NODES = ["SP1", "B3", "B2", "B1", "BASE"]
ROUTES = {"SP1": ["B3", "B2", "B1"], "B3": ["B2", "B1"], "B2": ["B1"], "B1": [], "BASE": []}
ZONES = {"SP1": "B3", "B3": "B3", "B2": "B2", "B1": "B1", "BASE": "BASE"}
SOS_TEXTS = [
    "2 of us, leg injured",
    "trapped under desk near stairs, can't move left arm",
    "my dad is having trouble breathing",
    "4 people here, we can hear you",
    "water coming in, need help fast",
    "kid with me, she is ok but scared",
]


def _msg(rng, t, src, b2_down, **fields):
    m = make_msg(t, src, 4, z=ZONES[src], **fields)
    m["rt"] = [n for n in ROUTES[src] if not (b2_down and n == "B2")]   # mesh routes around a dead B2
    m["ttl"] = 4 - len(m["rt"])
    return m


def _event(rng, kind, b2_down):
    if kind == "PERSON":
        th = 1 if rng.random() < 0.8 else 0
        return _msg(rng, "PERSON", "SP1", b2_down, c=rng.randint(72, 95) if th else rng.randint(55, 75), th=th,
                    v=rng.choice("LCR"), dt=rng.randint(6, 11) if th else rng.randint(2, 4))
    if kind == "HEAT":
        return _msg(rng, "HEAT", "SP1", b2_down, c=40, v=rng.choice("LCR"), dt=rng.randint(5, 9))
    if kind == "PHONE":
        n = rng.randint(1, 4)
        return _msg(rng, "PHONE", "B3", b2_down, n=n, rssi=-rng.randint(55, 92),
                    ap=rng.randint(0, n) if rng.random() < 0.8 else None)
    if kind == "SOS":
        return None if b2_down else _msg(rng, "SOS", "B2", b2_down, msg=rng.choice(SOS_TEXTS))
    if kind == "HAZARD":
        return _msg(rng, "HAZARD", "B1", b2_down, msg="gas sensor above threshold")


def stream(speed=1.0, seed=None):
    """Yields JSON lines (str, no newline) forever, sleeping to keep real time / speed."""
    rng = random.Random(seed)
    every = {"PERSON": (15, 35), "HEAT": (30, 60), "PHONE": (20, 40), "SOS": (45, 90), "HAZARD": (150, 300)}
    next_ev = {"PERSON": 3, "PHONE": 8, "SOS": 14, "HEAT": 22, "HAZARD": 100}
    next_hb = {n: rng.uniform(0.5, 4) for n in NODES}
    outage = (70.0, 150.0)          # B2 silent for 80 s: amber after 30 s, grey after 60 s
    start, t, step = time.monotonic(), 0.0, 0.5
    while True:
        t += step
        delay = start + t / speed - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        if t >= outage[1]:
            begin = t + rng.uniform(90, 150)
            outage = (begin, begin + 80)
        b2_down = outage[0] <= t < outage[1]

        out = []
        for node in NODES:
            if t >= next_hb[node]:
                next_hb[node] = t + rng.uniform(13, 17)
                if not (node == "B2" and b2_down):
                    out.append(_msg(rng, "HB", node, b2_down))
        for kind in next_ev:
            if t >= next_ev[kind]:
                next_ev[kind] = t + rng.uniform(*every[kind])
                m = _event(rng, kind, b2_down)
                if m:
                    out.append(m)

        for m in out:
            line = encode_line(m).decode("utf-8").rstrip("\n")
            yield line
            if rng.random() < 0.12:                    # flooding delivered it twice
                yield line
        if rng.random() < 0.01:
            yield "[LoRa] rx rssi=-87 snr=7.5"        # firmware debug noise: must be ignored


def main():
    p = argparse.ArgumentParser(description="Print a fake Rescue Net message stream")
    p.add_argument("--speed", type=float, default=1.0, help="time multiplier, e.g. 5 = five times faster")
    p.add_argument("--seed", type=int)
    args = p.parse_args()
    try:
        for line in stream(args.speed, args.seed):
            print(line, flush=True)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


if __name__ == "__main__":
    main()
