"""Dashboard backend: battery / LoRa-only from HB, SOS phone listen readings (SL/SLR), PLACE cards, /api/cmd."""
import json

import pytest

from dashboard import app as dash


@pytest.fixture
def state(monkeypatch):
    monkeypatch.setattr(dash.jsonlog, "append", lambda *a, **k: None)
    s = dash.State()
    s.sent = []
    s.sender = s.sent.append
    return s


def feed(state, obj, now=None, monkeypatch=None):
    dash.handle_line(state, json.dumps(obj), record=False)


def test_heartbeat_battery_and_lora_flag(state):
    feed(state, {"t": "HB", "src": "B1", "bt": 87, "bv": 3.95})
    feed(state, {"t": "HB", "src": "B2", "lo": 1})
    nodes = state.snapshot()["nodes"]
    assert nodes["B1"]["bt"] == 87 and nodes["B1"]["bv"] == 3.95 and nodes["B1"]["lo"] == 0
    assert nodes["B2"]["bt"] is None and nodes["B2"]["bv"] is None and nodes["B2"]["lo"] == 1
    assert "bt" not in nodes["BASE"]
    feed(state, {"t": "HB", "src": "B1"})                 # gauge gone: unknown again
    assert state.snapshot()["nodes"]["B1"]["bt"] is None
    assert state.snapshot()["alerts"] == []


def test_heartbeat_wrapped_in_lora_rx(state):
    feed(state, {"lora_rx": {"t": "HB", "src": "B2", "bt": 15, "bv": 3.4, "lo": 1}, "rssi": -90})
    assert state.snapshot()["nodes"]["B2"]["bt"] == 15


def test_place_is_info_card_folded_and_needs_no_src(state):
    feed(state, {"t": "PLACE", "z": "B1", "up": "BASE", "r": -74})
    feed(state, {"t": "PLACE", "z": "B1", "up": "BASE", "r": -77})
    alerts = state.snapshot()["alerts"]
    assert len(alerts) == 1
    a = alerts[0]
    assert a["t"] == "PLACE" and a["handled"] is True and a["r"] == -77 and a["src"] == "B1" and a["reps"] == 2
    feed(state, {"t": "PLACE", "z": "B2", "up": "B1", "lost": 1})
    assert len(state.snapshot()["alerts"]) == 2


def test_sl_slr_stored_not_as_cards_and_matched_to_sos(state):
    feed(state, {"t": "SOS", "src": "B1", "id": "a1", "msg": "help", "sr": -58})
    sos_key = state.snapshot()["alerts"][0]["key"]
    feed(state, {"t": "SL", "z": "B1", "pm": "A1B2C3", "r": -58})
    feed(state, {"t": "SLR", "z": "B2", "pm": "a1b2c3", "r": -71, "n": 12})
    feed(state, {"t": "SLR", "z": "BASE", "pm": "a1b2c3", "n": 0})
    snap = state.snapshot()
    assert [a["t"] for a in snap["alerts"]] == ["SOS"]
    (e,) = snap["sl"]
    assert e["pm"] == "a1b2c3" and e["sos_key"] == sos_key
    assert e["r"] == {"B1": -58, "B2": -71}
    assert e["n"] == {"B2": 12, "BASE": 0}


def test_sl_newest_first_and_expiry(state, monkeypatch):
    t = [1000.0]
    monkeypatch.setattr(dash.time, "time", lambda: t[0])
    feed(state, {"t": "SL", "z": "B1", "pm": "000001", "r": -60})
    t[0] += 5
    feed(state, {"t": "SL", "z": "B1", "pm": "000002", "r": -62})
    assert [e["pm"] for e in state.snapshot()["sl"]] == ["000002", "000001"]
    assert state.snapshot()["sl"][0]["sos_key"] is None          # no SOS card to match
    t[0] += dash.SL_KEEP_S - 2
    assert [e["pm"] for e in state.snapshot()["sl"]] == ["000002"]
    t[0] += 10
    feed(state, {"t": "SL", "z": "B1", "pm": "000003", "r": -50})
    assert [e["pm"] for e in state.snapshot()["sl"]] == ["000003"]
    assert list(state.sl) == ["000003"]


def test_sl_rejects_bad_readings(state):
    feed(state, {"t": "SL", "z": "B1", "pm": "abc", "r": 5})
    feed(state, {"t": "SLR", "z": "B2", "r": -70, "n": 3})       # no pm
    (e,) = state.snapshot()["sl"]
    assert e["r"] == {}


def test_cmd_validation_and_send(state):
    assert state.command("B3", "lora")[0] == 400
    assert state.command("B2", "reboot")[0] == 400
    assert state.sent == []
    assert state.command("B2", "lora") == (200, "sent")
    assert json.loads(state.sent[-1]) == {"t": "CMD", "to": "B2", "c": "lora"}
    m = state.snapshot()["mode"]
    assert m["lora_only"] is True and m["confirmed"] is False and m["sent"]
    feed(state, {"t": "HB", "src": "B2"})                        # still bulk: not yet
    assert state.snapshot()["mode"]["confirmed"] is False
    feed(state, {"t": "HB", "src": "B2", "lo": 1})
    assert state.snapshot()["mode"]["confirmed"] is True
    assert state.command("B2", "bulk")[0] == 200
    m = state.snapshot()["mode"]
    assert m["lora_only"] is False and m["confirmed"] is False
    feed(state, {"t": "HB", "src": "B2", "lo": 0})
    assert state.snapshot()["mode"]["confirmed"] is True
    assert state.command("B1", "off")[0] == 200
    assert state.snapshot()["mode"]["lora_only"] is False           # off/on does not touch the mode
    assert state.snapshot()["last_cmd"]["c"] == "off"


def test_cmd_without_sender(state):
    state.sender = None
    assert state.command("B2", "lora")[0] == 409


def test_api_cmd(state):
    client = dash.create_app(state).test_client()
    r = client.post("/api/cmd", json={"to": "B2", "c": "lora"})
    assert r.status_code == 200 and r.get_json()["mode"]["lora_only"] is True
    assert client.post("/api/cmd", json={"to": "BASE", "c": "lora"}).status_code == 400
    assert client.post("/api/cmd", data="x").status_code == 400
    snap = client.get("/api/state").get_json()
    assert snap["mode"]["lora_only"] is True and snap["sl"] == []


def test_repeat_sl_keeps_sos_card_and_history(state, monkeypatch):
    t = [1000.0]
    monkeypatch.setattr(dash.time, "time", lambda: t[0])
    feed(state, {"t": "SOS", "src": "B1", "id": "a1", "msg": "help", "sr": -58})
    sos_key = state.snapshot()["alerts"][0]["key"]
    for i, (r1, r2) in enumerate([(-58, -70), (-60, -72), (-56, -74)]):
        t[0] = 1000.0 + 20 * i                       # B1 repeats SL every 20 s while the phone polls /status
        feed(state, {"t": "SL", "z": "B1", "pm": "a1b2c3", "r": r1})
        feed(state, {"t": "SLR", "z": "B2", "pm": "a1b2c3", "r": r2, "n": 5})
    (e,) = state.snapshot()["sl"]
    assert e["sos_key"] == sos_key                   # later SLs (no SOS within 10 s) stay on the same card
    assert e["r"] == {"B1": -56, "B2": -74}          # newest round
    assert e["h"] == {"B1": [-58, -60, -56], "B2": [-70, -72, -74]}
    for i in range(10):
        feed(state, {"t": "SLR", "z": "B2", "pm": "a1b2c3", "r": -80 - i, "n": 5})
    assert len(state.snapshot()["sl"][0]["h"]["B2"]) == dash.SL_HIST
