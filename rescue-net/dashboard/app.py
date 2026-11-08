"""Command post: read JSON lines from the base ESP32 (or stdin / mock / replay), keep state, serve the UI.

python -m dashboard.app [--port /dev/cu.usbserial-XXXX | COM3] [--stdin] [--mock] [--replay FILE] [--http-port 5050]
python -m dashboard.app --hub http://127.0.0.1:8765 --token SECRET   (BASE owned by tools/board_hub.py)
"""
import argparse
import json
import logging
import sys
import threading
import time
import urllib.parse
import urllib.request
from collections import OrderedDict
from pathlib import Path

from flask import Flask, abort, jsonify, request, send_from_directory

from common import jsonlog
from common.protocol import extract_message, parse_line

log = logging.getLogger("dashboard")

KNOWN_NODES = ["SP1", "B2", "B1", "BASE"]     # our build: 2 breadcrumbs (B2 LoRa gateway, B1 relay + portal)
GREEN_S, AMBER_S = 30, 60
DEDUP_WINDOW_S = 300        # 4-hex IDs repeat eventually; flooding duplicates arrive within seconds
MAX_ALERTS = 500
FOLD_S = 60                 # repeats of the same report (type + source + zone) within 60 s update one card
FOLD_TYPES = ("PERSON", "HEAT", "PHONE")
ACK_DEFAULT = "Help is on the way"
STATIC = Path(__file__).resolve().parent / "static"


def priority(a):
    """Unhandled SOS first, then a person on the ground, then everything else (newest first within each)."""
    if a.get("t") == "SOS":
        return 2
    return 1 if a.get("t") == "PERSON" and a.get("ps") == "D" else 0


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.seen = OrderedDict()                 # (src, id) -> first rx time
        self.nodes = {n: None for n in KNOWN_NODES}
        self.alerts = []
        self.seq = 0
        self.connected = False
        self.source = ""
        self.stats = {"received": 0, "duplicates": 0, "ignored": 0}
        self.sender = None                        # callable(line: str) -> None, set by a source that can write

    def set_source(self, connected, source):
        with self.lock:
            self.connected, self.source = connected, source

    def count_ignored(self):
        with self.lock:
            self.stats["ignored"] += 1

    def mark_seen(self, node, now):
        with self.lock:
            self.nodes[node] = now

    def ingest(self, msg, now):
        """Returns False for a duplicate (same src + id seen in the last 5 minutes)."""
        with self.lock:
            while self.seen and now - next(iter(self.seen.values())) > DEDUP_WINDOW_S:
                self.seen.popitem(last=False)
            key = (msg.get("src"), msg.get("id"))
            if msg.get("id") is not None:
                if key in self.seen:
                    self.stats["duplicates"] += 1
                    return False
                self.seen[key] = now
            self.stats["received"] += 1
            route = msg.get("rt") if isinstance(msg.get("rt"), list) else []
            for node in [msg.get("src"), *route]:
                if isinstance(node, str) and node:
                    self.nodes[node] = now
            t = msg.get("t")
            if t in ("HB", "ACK"):                # ACK is our own reply going down the chain
                return True
            if t in FOLD_TYPES:
                for a in reversed(self.alerts):
                    if (not a["handled"] and a.get("t") == t and a.get("src") == msg.get("src")
                            and a.get("z") == msg.get("z") and a.get("ps") == msg.get("ps")
                            and now - a["rx"] < FOLD_S):
                        reps, first, key = a.get("reps", 1) + 1, a.get("first", a["rx"]), a["key"]
                        a.clear()
                        a.update({**msg, "key": key, "rx": now, "handled": False,
                                  "reps": reps, "first": first})
                        return True
            self.seq += 1
            self.alerts.append({**msg, "key": str(self.seq), "rx": now, "handled": False})
            del self.alerts[:-MAX_ALERTS]
            return True

    def handle(self, key):
        with self.lock:
            for a in self.alerts:
                if a["key"] == key:
                    a["handled"] = True
                    return True
        return False

    def ack(self, key, text):
        """Send an ACK for an SOS card down the chain (BASE -> B1, which shows it on the survivor's phone)."""
        with self.lock:
            a = next((a for a in self.alerts if a["key"] == key), None)
            sender = self.sender
        if a is None or a.get("t") != "SOS" or not a.get("id"):
            return 404, "no such SOS"
        if sender is None:
            return 409, "this source cannot send (use --port or --hub)"
        line = json.dumps({"t": "ACK", "ref": a["id"], "msg": text}, separators=(",", ":"))
        try:
            sender(line)
        except Exception as e:
            log.warning("ACK send failed: %s", e)
            return 502, f"send failed: {e}"
        log.info("ACK sent %s", line)
        with self.lock:
            a["acked"], a["ack_msg"], a["handled"] = time.time(), text, True
        return 200, "sent"

    def snapshot(self):
        now = time.time()
        with self.lock:
            nodes = {}
            for name, seen in self.nodes.items():
                age = None if seen is None else now - seen
                health = "grey" if age is None or age >= AMBER_S else ("green" if age < GREEN_S else "amber")
                nodes[name] = {"last_seen": seen, "age": age, "health": health}
            alerts = sorted(self.alerts, key=lambda a: (a["handled"], -priority(a), -a["rx"]))
            return {"now": now, "nodes": nodes, "alerts": alerts, "serial_connected": self.connected,
                    "source": self.source, "stats": dict(self.stats), "can_send": self.sender is not None}



def handle_line(state, raw, record=True):
    msg = extract_message(parse_line(raw))
    if msg is None:                   # non-JSON, other teams' packets, or board status like {"sent":..}
        text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else raw
        if text.strip():
            state.count_ignored()
            log.debug("ignored: %s", text.strip()[:120])
        return
    now = time.time()
    if record:
        jsonlog.append("messages", {"rx": round(now, 3), "msg": msg})
    new = state.ingest(msg, now)
    log.info("%s %s", "RX " if new else "DUP", json.dumps(msg, separators=(",", ":")))


# ---- input sources (each runs in its own daemon thread) ----

def read_serial(state, port, baud):
    import serial
    from common import serialport
    warned = False
    while True:
        device = port or serialport.find_port()
        if not device:
            if not warned:
                log.warning("no serial port found; plug in the base ESP32 (retrying every 2 s)")
                warned = True
            state.set_source(False, "no serial port")
            time.sleep(2)
            continue
        try:
            ser = serialport.open_port(device, baud)
        except (serial.SerialException, OSError) as e:
            log.warning("cannot open %s: %s (retrying)", device, e)
            state.set_source(False, device)
            time.sleep(2)
            continue
        warned = False
        log.info("serial connected: %s @ %d", device, baud)
        state.set_source(True, device)
        state.sender = lambda line, ser=ser: (ser.write(line.encode("utf-8") + b"\n"), ser.flush())
        try:
            while True:
                raw = ser.readline()
                if raw:
                    state.mark_seen("BASE", time.time())    # the base board itself is alive
                    handle_line(state, raw)
        except (serial.SerialException, OSError) as e:
            log.warning("serial lost (%s), reconnecting", e)
        finally:
            state.sender = None
            state.set_source(False, device)
            try:
                ser.close()
            except Exception:
                pass
        time.sleep(1)


def read_stdin(state):
    state.set_source(True, "stdin")
    for raw in sys.stdin.buffer:
        handle_line(state, raw)
    log.info("stdin closed")
    state.set_source(False, "stdin (ended)")


def read_hub(state, url, token, label="BASE"):
    """Follow the base board through tools/board_hub.py (it owns the COM port): long-poll /log, write via /send."""
    url = url.rstrip("/")
    hdr = {"X-Token": token}

    def get(path):
        req = urllib.request.Request(url + path, headers=hdr)
        with urllib.request.urlopen(req, timeout=35) as r:
            return json.load(r)

    def send(line):
        req = urllib.request.Request(f"{url}/send/{label}", data=line.encode("utf-8"), headers=hdr, method="POST")
        with urllib.request.urlopen(req, timeout=10) as r:
            r.read()

    seq = None
    while True:
        try:
            if seq is None:
                seq = get(f"/log/{label}?n=0")["seq"]
                log.info("hub connected: %s (%s)", url, label)
                state.sender = send
                state.set_source(True, f"hub {label}")
            d = get(f"/log/{label}?since={seq}&wait=25")
            for _seq, _ts, text in d.get("lines", []):
                state.mark_seen(label, time.time())
                handle_line(state, text)
            seq = d.get("seq", seq)
        except Exception as e:
            if seq is not None or state.sender is not None:
                log.warning("hub lost (%s), retrying", e)
            seq, state.sender = None, None
            state.set_source(False, f"hub {url}")
            time.sleep(2)


def read_mock(state, speed):
    from dashboard import mock_feed
    state.set_source(True, "mock feed")
    for line in mock_feed.stream(speed):
        handle_line(state, line, record=False)


def read_replay(state, path, speed):
    """Re-play a messages-*.jsonl log at real speed (gaps capped at 10 s). Plain message lines work too."""
    state.set_source(True, f"replay {Path(path).name}")
    prev = None
    with open(path, "rb") as f:
        for raw in f:
            obj = parse_line(raw)
            if obj is None:
                continue
            rx, msg = (obj.get("rx"), obj["msg"]) if isinstance(obj.get("msg"), dict) else (None, obj)
            gap = 1.0 if rx is None or prev is None else min(max(rx - prev, 0.0), 10.0)
            prev = rx
            time.sleep(gap / speed)
            handle_line(state, json.dumps(msg), record=False)
    log.info("replay finished")
    state.set_source(True, f"replay {Path(path).name} (finished)")   # blue "info" pill, not a red error


# ---- web ----

def create_app(state):
    app = Flask(__name__, static_folder=None)

    @app.get("/")
    def index():
        resp = send_from_directory(STATIC, "index.html")
        resp.headers["Cache-Control"] = "no-store"
        return resp

    @app.get("/api/state")
    def api_state():
        return jsonify(state.snapshot())

    @app.post("/api/ack/<key>")
    def api_ack(key):
        body = request.get_json(silent=True) or {}
        text = " ".join(str(body.get("msg") or ACK_DEFAULT).split())[:40]   # firmware keeps <= 40 bytes
        code, why = state.ack(key, text)
        return jsonify(ok=code == 200, why=why), code

    @app.post("/api/handle/<key>")
    def api_handle(key):
        if not state.handle(key):
            abort(404)
        return jsonify(ok=True)

    return app


def main():
    p = argparse.ArgumentParser(description="Rescue Net command post")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--port", help="serial port of the base ESP32 (default: auto-detect)")
    src.add_argument("--stdin", action="store_true", help="read JSON lines from stdin")
    src.add_argument("--mock", action="store_true", help="built-in fake message stream")
    src.add_argument("--replay", metavar="FILE", help="re-play a logs/messages-*.jsonl file")
    src.add_argument("--hub", metavar="URL", help="read BASE through tools/board_hub.py, e.g. http://127.0.0.1:8765")
    p.add_argument("--token", help="board hub token (--hub); default: env BOARD_HUB_TOKEN")
    p.add_argument("--speed", type=float, default=1.0, help="speed-up for --mock / --replay")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--http-port", type=int, default=5050, help="default 5050 (macOS AirPlay uses 5000)")
    p.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to allow other machines")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, stream=sys.stderr,
                        format="%(asctime)s %(name)-9s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("werkzeug").setLevel(logging.WARNING)      # no log line per 1 s poll

    state = State()
    if args.stdin:
        target, targs = read_stdin, (state,)
    elif args.mock:
        target, targs = read_mock, (state, args.speed)
    elif args.hub:
        import os
        token = args.token or os.environ.get("BOARD_HUB_TOKEN")
        if not token:
            p.error("--hub needs --token or BOARD_HUB_TOKEN")
        target, targs = read_hub, (state, args.hub, token)
    elif args.replay:
        target, targs = read_replay, (state, args.replay, args.speed)
    else:
        target, targs = read_serial, (state, args.port, args.baud)
    threading.Thread(target=target, args=targs, name="reader", daemon=True).start()

    log.info("dashboard on http://%s:%d/", "localhost" if args.host == "127.0.0.1" else args.host, args.http_port)
    create_app(state).run(host=args.host, port=args.http_port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
