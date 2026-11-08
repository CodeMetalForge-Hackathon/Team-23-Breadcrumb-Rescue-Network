from spotter.debounce import Debouncer, RateLimiter

P_C = ("PERSON", "C")
H_R = ("HEAT", "R")


def run(d, frames, t0=0.0, dt=0.1):
    fired = []
    for i, keys in enumerate(frames):
        fired += [(i, k) for k in d.update(keys, t0 + i * dt)]
    return fired


def test_needs_3_of_last_5():
    d = Debouncer()
    assert run(d, [{P_C}, set(), {P_C}, set(), set()]) == []
    d = Debouncer()
    assert run(d, [{P_C}, set(), {P_C}, set(), {P_C}]) == [(4, P_C)]


def test_old_frames_fall_out_of_window():
    d = Debouncer()
    assert run(d, [{P_C}, {P_C}, set(), set(), set(), set(), {P_C}]) == []


def test_cooldown_per_key():
    d = Debouncer(cooldown_s=10)
    fired = run(d, [{P_C}] * 200, dt=0.1)     # 20 s of continuous detection
    assert [i for i, _ in fired] == [2, 102]


def test_keys_are_independent():
    d = Debouncer()
    fired = run(d, [{P_C, H_R}, {P_C}, {P_C, H_R}, {H_R}])
    assert fired == [(2, P_C), (3, H_R)]


def test_cooldown_change_takes_effect():
    d = Debouncer(cooldown_s=10)
    run(d, [{P_C}] * 3)
    d.cooldown_s = 1
    assert d.update({P_C}, 1.5) == [P_C]


def test_rate_limiter_one_per_second():
    r = RateLimiter(1.0)
    sent = []
    for i in range(30):                        # 3 s at 10 Hz
        now = i * 0.1
        if r.ready(now):
            r.mark(now)
            sent.append(round(now, 1))
    assert sent == [0.0, 1.0, 2.0]
