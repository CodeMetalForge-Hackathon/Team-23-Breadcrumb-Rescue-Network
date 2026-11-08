"""Command post: read JSON lines from the base ESP32 (or stdin / mock / replay), keep state, serve the UI.

python -m dashboard.app [--port /dev/cu.usbserial-XXXX | COM3] [--stdin] [--mock] [--replay FILE] [--http-port 5050]
"""
import argparse
import json
import logging
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path

from flask import Flask, abort, jsonify, send_from_directory

from common import jsonlog
from common.protocol import extract_message, parse_line

log = logging.getLogger("dashboard")

KNOWN_NODES = ["SP1", "B3", "B2", "B1", "BASE"]
GREEN_S, AMBER_S = 30, 60
DEDUP_WINDOW_S = 300        # 4-hex IDs repeat eventually; flooding duplicates arrive within seconds
MAX_ALERTS = 500
STATIC = Path(__file__).resolve().parent / "static"


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
            if msg.get("t") != "HB":
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

    def snapshot(self):
        now = time.time()
        with self.lock:
            nodes = {}
            for name, seen in self.nodes.items():
                age = None if seen is None else now - seen
                health = "grey" if age is None or age >= AMBER_S else ("green" if age < GREEN_S else "amber")
                nodes[name] = {"last_seen": seen, "age": age, "health": health}
            alerts = sorted(self.alerts, key=lambda a: (a["handled"], -a["rx"]))
            return {"now": now, "nodes": nodes, "alerts": alerts, "serial_connected": self.connected,
                    "source": self.source, "stats": dict(self.stats)}


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
        try:
            while True:
                raw = ser.readline()
                if raw:
                    state.mark_seen("BASE", time.time())    # the base board itself is alive
                    handle_line(state, raw)
        except (serial.SerialException, OSError) as e:
            log.warning("serial lost (%s), reconnecting", e)
        finally:
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
    elif args.replay:
        target, targs = read_replay, (state, args.replay, args.speed)
    else:
        target, targs = read_serial, (state, args.port, args.baud)
    threading.Thread(target=target, args=targs, name="reader", daemon=True).start()

    log.info("dashboard on http://%s:%d/", "localhost" if args.host == "127.0.0.1" else args.host, args.http_port)
    create_app(state).run(host=args.host, port=args.http_port, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
