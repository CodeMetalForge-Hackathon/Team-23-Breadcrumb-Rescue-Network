from spotter.linkwatch import LinkWatch


def make(replies, up_ok=True):
    t = [0.0]
    calls = []
    lw = LinkWatch(ping_fn=lambda h: replies.pop(0), up_fn=lambda c: (calls.append(c), (up_ok, ["x"]))[1],
                   clock=lambda: t[0], cooldown_s=20)
    return lw, t, calls


def test_bounces_after_three_misses_then_holds_off():
    lw, t, calls = make([False, False, False, False])
    assert [lw.step() for _ in range(3)] == [False, False, True]
    assert calls == ["RESCUE-LINK"] and lw.status == "rejoined"
    t[0] = 10.0
    assert lw.step() is False and len(calls) == 1      # inside the cooldown: no ping, no bounce


def test_one_reply_resets_the_count():
    lw, t, calls = make([False, False, True, False, False])
    for _ in range(5):
        lw.step()
    assert calls == [] and lw.misses == 2


def test_failed_rejoin_is_reported():
    lw, t, calls = make([False] * 3, up_ok=False)
    for _ in range(3):
        lw.step()
    assert lw.status.startswith("rejoin failed")


def test_failed_rejoin_retries_on_the_next_check():
    lw, t, calls = make([False] * 5, up_ok=False)
    assert [lw.step() for _ in range(5)] == [False, False, True, True, True]   # no cooldown while B2 is gone
    assert len(calls) == 3
