#!/usr/bin/env python3
"""Board Hub: share the USB dev boards plugged into this laptop with agents on other machines.

Runs on the laptop (Windows). One process owns every board's serial port; agents anywhere on
the tailnet use HTTP, so several agents never fight over a COM port and USB re-enumeration
during flashing is handled locally.

  python board_hub.py --token SECRET [--port 8765] [--esptool PATH]

API (header "X-Token: SECRET", or ?token=SECRET):
  GET  /boards                         list boards: label, port, vid:pid, mac/serial, last line
  GET  /log/<label>?n=100              last n serial lines   ({"seq":..,"lines":[[seq,ts,text],..]})
  GET  /log/<label>?since=SEQ&wait=20  lines after SEQ, long-polls up to `wait` s (for tailing)
  POST /send/<label>                   body = one text line, written to the board's serial
  POST /flash/<label>?kind=esp32       body = merged .bin (written at 0x0)  -> esptool
  POST /flash/<label>?kind=rp2040      body = .uf2 (1200-baud touch, copy to RPI-RP2 drive)
  POST /reset/<label>                  ESP32: reset via esptool
  POST /label?port=COM4&name=CP        name a port by hand (persisted; also learned from boot lines)

Labels are learned automatically from {"boot":"CP","mac":"..."} lines and remembered by MAC or
USB serial number in board_hub_labels.json, so they survive COM-number changes.
"""
import argparse, collections, glob, ipaddress, json, os, re, shutil, subprocess, sys, tempfile, threading, time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs, unquote

import serial
from serial.tools import list_ports

BOARD_VIDS = {0x303A, 0x239A, 0x2E8A, 0x10C4, 0x1A86}  # Espressif, Adafruit, RPi, CP210x, CH340
ESP_ROM_PID = 0x1001                                   # ESP32-S3 USB-Serial-JTAG (download mode)
HERE = os.path.dirname(os.path.abspath(__file__))
LABELS_FILE = os.path.join(HERE, "board_hub_labels.json")
LOG_DIR = os.path.join(HERE, "board_hub_logs")


def norm_key(s):
    """MAC or USB serial -> stable key ('98:c3:77:62:67:24' and '98C377626724' map the same)."""
    h = re.sub(r"[^0-9A-Fa-f]", "", s or "").upper()
    return h[-12:] if len(h) >= 12 and re.fullmatch(r"[0-9A-F]+", h) else (s or "")


class Board:
    def __init__(self, info):
        self.dev = info.device
        self.vid, self.pid = info.vid, info.pid
        self.sn = info.serial_number or ""
        self.key = norm_key(self.sn)
        self.desc = info.description
        self.lines = collections.deque(maxlen=3000)
        self.seq = 0
        self.ser = None
        self.stop = threading.Event()
        self.cond = threading.Condition()

    @property
    def is_esp(self):
        return self.vid == 0x303A or (self.vid == 0x239A and self.desc and "S3" in self.desc)


class Hub:
    def __init__(self, esptool):
        self.esptool = esptool
        if os.path.isfile(esptool) or shutil.which(esptool):
            self.esptool_cmd = [esptool]
        else:  # fall back to the pip package:  py -m pip install esptool
            self.esptool_cmd = [sys.executable, "-m", "esptool"]
        self.boards = {}            # dev -> Board
        self.labels = {}            # key -> label
        self.lock = threading.RLock()
        self.flashing = set()       # devs reserved by a flash/reset job
        self.global_flash = threading.Lock()
        if os.path.exists(LABELS_FILE):
            self.labels = json.load(open(LABELS_FILE))
        os.makedirs(LOG_DIR, exist_ok=True)

    # ---- labels ----
    def label_of(self, b):
        return self.labels.get(b.key) or self.labels.get(b.dev)

    def set_label(self, key, name):
        with self.lock:
            self.labels[key] = name
            json.dump(self.labels, open(LABELS_FILE, "w"), indent=1)

    def find(self, label):
        with self.lock:
            for b in self.boards.values():
                if self.label_of(b) == label or b.dev.lower() == label.lower():
                    return b
        return None

    # ---- serial readers ----
    def on_line(self, b, text):
        bt = text[2:] if text.startswith("# {") else text   # BASE prefixes debug lines with '#'
        if bt.startswith('{"boot"'):
            try:
                j = json.loads(bt)
                name = j.get("boot")
                if name:
                    self.set_label(b.key, name)
                    if j.get("mac"):
                        self.set_label(norm_key(j["mac"]), name)
            except ValueError:
                pass
        ts = time.strftime("%H:%M:%S")
        with b.cond:
            b.seq += 1
            b.lines.append((b.seq, ts, text))
            b.cond.notify_all()
        name = self.label_of(b) or b.dev
        with open(os.path.join(LOG_DIR, f"{name}.log"), "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d')} {ts} {text}\n")

    def reader(self, b):
        buf = b""
        while not b.stop.is_set():
            try:
                if b.ser is None:
                    s = serial.Serial()
                    s.port, s.baudrate, s.timeout = b.dev, 115200, 0.2
                    s.dtr, s.rts = True, False   # USB CDC only sends with DTR; RTS high could reset
                    s.open()
                    b.ser = s
                data = b.ser.read(1024)
            except Exception:
                self.close(b)
                time.sleep(1)
                continue
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.on_line(b, line.decode("utf-8", "replace").rstrip("\r"))
        self.close(b)

    def close(self, b):
        try:
            if b.ser:
                b.ser.close()
        except Exception:
            pass
        b.ser = None

    def detach(self, b):
        """Stop the reader so another tool can open the port."""
        b.stop.set()
        time.sleep(0.4)
        self.close(b)
        with self.lock:
            self.boards.pop(b.dev, None)
            self.flashing.add(b.dev)

    def scan_forever(self):
        while True:
            try:
                present = {p.device: p for p in list_ports.comports() if p.vid in BOARD_VIDS}
                with self.lock:
                    for dev in list(self.boards):
                        if dev not in present:
                            self.boards.pop(dev).stop.set()
                    for dev, info in present.items():
                        if dev in self.boards or dev in self.flashing:
                            continue
                        if info.vid == 0x303A and info.pid == ESP_ROM_PID:
                            continue  # board in download mode: leave it for esptool
                        b = Board(info)
                        self.boards[dev] = b
                        threading.Thread(target=self.reader, args=(b,), daemon=True).start()
            except Exception as e:
                print("scan error:", e, file=sys.stderr)
            time.sleep(1)

    # ---- flashing ----
    def run(self, cmd, timeout=180):
        if cmd[0] == self.esptool:
            cmd = self.esptool_cmd + cmd[1:]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, stdin=subprocess.DEVNULL,
                           creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        return r.returncode, (r.stdout + r.stderr)[-4000:]

    def wait_port(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for p in list_ports.comports():
                if pred(p):
                    return p
            time.sleep(0.5)
        return None

    def kick_rom(self, key, log):
        """If the board is stuck in ROM download mode, reset it into the app."""
        p = self.wait_port(lambda p: p.vid == 0x303A and p.pid == ESP_ROM_PID and norm_key(p.serial_number) == key, 3)
        if p:
            rc, out = self.run([self.esptool, "--chip", "esp32s3", "--port", p.device,
                                "--before", "no-reset", "--after", "hard-reset", "read-mac"], 60)
            log.append(f"[hub] board was in ROM mode on {p.device}; hard reset rc={rc}")

    def enter_rom(self, b, log):
        """Put an ESP32-S3 running TinyUSB CDC into ROM download mode; return the new ROM port or None.
        The chip re-enumerates (new COM number), so esptool must be pointed at the new port."""
        key = b.key
        try:
            s = serial.Serial(b.dev, 1200)   # 1200-baud touch -> download mode (Arduino ESP32 TinyUSB)
            s.dtr = False
            s.close()
        except Exception as e:
            log.append(f"[hub] 1200-baud touch on {b.dev}: {e}")
        p = self.wait_port(lambda p: p.vid == 0x303A and p.pid == ESP_ROM_PID and norm_key(p.serial_number) == key, 8)
        if p:
            log.append(f"[hub] download mode on {p.device}")
            return p.device
        # second try: esptool's own DTR/RTS reset on the app port (it will error when the port vanishes)
        rc, out = self.run([self.esptool, "--chip", "esp32s3", "--port", b.dev, "--before", "default-reset",
                            "--after", "no-reset", "chip-id"], 30)
        p = self.wait_port(lambda p: p.vid == 0x303A and p.pid == ESP_ROM_PID and norm_key(p.serial_number) == key, 8)
        if p:
            log.append(f"[hub] download mode on {p.device} (esptool reset)")
            return p.device
        log.append("[hub] board did not enter download mode (try: hold BOOT/D0, tap RESET)")
        return None

    def flash_esp(self, b, data):
        log = []
        key, label = b.key, self.label_of(b)
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
            f.write(data)
            path = f.name
        rc = 1
        try:
            self.detach(b)
            rom = self.enter_rom(b, log)
            if rom:
                with self.lock:
                    self.flashing.add(rom)
                rc, out = self.run([self.esptool, "--chip", "esp32s3", "--port", rom, "--baud", "921600",
                                    "--before", "no-reset", "--after", "hard-reset",
                                    "write-flash", "-z", "0x0", path])
                log.append(out)
                with self.lock:
                    self.flashing.discard(rom)
            self.kick_rom(key, log)
        finally:
            os.unlink(path)
            with self.lock:
                self.flashing.discard(b.dev)
        p = self.wait_port(lambda p: norm_key(p.serial_number) == key and p.pid != ESP_ROM_PID, 15)
        log.append(f"[hub] {label} back on {p.device}" if p else f"[hub] WARNING: {label} did not re-appear")
        return rc == 0, "\n".join(log)

    def reset_esp(self, b):
        log = []
        key = b.key
        try:
            self.detach(b)
            rc, out = self.run([self.esptool, "--chip", "esp32s3", "--port", b.dev,
                                "--after", "hard-reset", "read-mac"], 60)
            log.append(out)
            self.kick_rom(key, log)
        finally:
            with self.lock:
                self.flashing.discard(b.dev)
        return rc == 0, "\n".join(log)

    @staticmethod
    def uf2_drive():
        for d in "DEFGHIJKLMNOPQRSTUVWXYZ":
            if os.path.exists(f"{d}:\\INFO_UF2.TXT"):
                return f"{d}:\\"
        return None

    def flash_uf2(self, b, data):
        log = []
        if b is not None:
            key = b.key
            self.detach(b)
            try:
                for attempt in range(6):          # 1200-baud touch -> UF2 bootloader (Windows may need retries)
                    try:
                        s = serial.Serial()
                        s.port, s.baudrate, s.dtr = b.dev, 1200, True
                        s.open()
                        time.sleep(0.1)
                        s.dtr = False
                        s.close()
                        break
                    except Exception as e:
                        log.append(f"[hub] 1200-baud touch try {attempt + 1}: {e}")
                        time.sleep(0.7)
            finally:
                with self.lock:
                    self.flashing.discard(b.dev)
        else:
            key = None
        t0 = time.time()
        drive = None
        while time.time() - t0 < 15 and not drive:
            drive = self.uf2_drive()
            time.sleep(0.5)
        if not drive:
            return False, "\n".join(log + ["[hub] no RPI-RP2 drive appeared (hold BOOT, tap RESET)"])
        with open(os.path.join(drive, "firmware.uf2"), "wb") as f:
            f.write(data)
        log.append(f"[hub] copied {len(data)} bytes to {drive}")
        if key:
            p = self.wait_port(lambda p: norm_key(p.serial_number) == key, 15)
            log.append(f"[hub] back on {p.device}" if p else "[hub] WARNING: port did not re-appear")
        return True, "\n".join(log)


def make_handler(hub, token):
    tailnet = ipaddress.ip_network("100.64.0.0/10")

    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *a):
            sys.stderr.write("%s %s\n" % (self.client_address[0], fmt % a))

        def reply(self, code, obj):
            body = (json.dumps(obj, ensure_ascii=False) + "\n").encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def check(self):
            ip = ipaddress.ip_address(self.client_address[0])
            q = parse_qs(urlparse(self.path).query)
            ok_ip = ip.is_loopback or ip in tailnet
            ok_tok = self.headers.get("X-Token") == token or q.get("token", [None])[0] == token
            if not (ok_ip and ok_tok):
                self.reply(403, {"error": "forbidden"})
                return None
            return q

        def body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return self.rfile.read(n) if n else b""

        def do_GET(self):
            q = self.check()
            if q is None:
                return
            parts = [unquote(x) for x in urlparse(self.path).path.strip("/").split("/")]
            if parts[0] == "boards":
                with hub.lock:
                    out = [{"label": hub.label_of(b), "port": b.dev, "usb": f"{b.vid:04X}:{b.pid:04X}",
                            "key": b.key, "desc": b.desc, "open": b.ser is not None,
                            "last": b.lines[-1][2] if b.lines else None} for b in hub.boards.values()]
                return self.reply(200, {"boards": out, "flashing": sorted(hub.flashing), "esptool": hub.esptool_cmd,
                                        "uf2_drive": hub.uf2_drive()})
            if parts[0] == "log" and len(parts) == 2:
                b = hub.find(parts[1])
                if not b:
                    return self.reply(404, {"error": f"no board {parts[1]}"})
                since = int(q.get("since", [-1])[0])
                if since >= 0:
                    wait = min(float(q.get("wait", [0])[0]), 60)
                    with b.cond:
                        if b.seq <= since and wait > 0:
                            b.cond.wait(wait)
                        lines = [l for l in b.lines if l[0] > since]
                else:
                    n = int(q.get("n", [100])[0])
                    lines = list(b.lines)[-n:]
                return self.reply(200, {"label": parts[1], "port": b.dev, "seq": b.seq, "lines": lines})
            self.reply(404, {"error": "unknown path"})

        def do_POST(self):
            try:
                self._post()
            except Exception:
                import traceback
                tb = traceback.format_exc()
                print(tb, flush=True)
                try:
                    self.reply(500, {"ok": False, "output": tb})
                except Exception:
                    pass

        def admin_update(self, q, data):
            """POST /admin/update?sha=<sha256 of body>: replace board_hub.py and restart the hub.
            Remote code execution for anyone with the token -- enabled by the laptop owner on purpose."""
            import hashlib, py_compile
            want = (q.get("sha", [""])[0] or "").lower()
            got = hashlib.sha256(data).hexdigest()
            if not data or want != got:
                return self.reply(400, {"ok": False, "error": "sha mismatch", "got": got})
            me = os.path.abspath(__file__)
            new = me + ".new"
            with open(new, "wb") as f:
                f.write(data)
            try:
                py_compile.compile(new, doraise=True)
            except py_compile.PyCompileError as e:
                os.unlink(new)
                return self.reply(400, {"ok": False, "error": "does not compile", "output": str(e)})
            shutil.copy2(me, me + ".bak")      # previous version kept for rollback
            os.replace(new, me)
            self.reply(200, {"ok": True, "sha": got, "restarting": True})
            # restart_hub.ps1 runs the hub in a loop, so exiting is enough to load the new code
            threading.Timer(1.0, os._exit, [0]).start()

        def _post(self):
            q = self.check()
            if q is None:
                return
            parts = [unquote(x) for x in urlparse(self.path).path.strip("/").split("/")]
            data = self.body()
            if parts[0] == "label":
                port, name = q.get("port", [""])[0], q.get("name", [""])[0]
                b = hub.find(port)
                if not b or not name:
                    return self.reply(400, {"error": "need existing port and name"})
                hub.set_label(b.key or b.dev, name)
                return self.reply(200, {"ok": True, "port": b.dev, "label": name})
            if parts == ["admin", "update"]:
                return self.admin_update(q, data)
            if len(parts) != 2:
                return self.reply(404, {"error": "unknown path"})
            b = hub.find(parts[1])
            if parts[0] == "send":
                if not b or not b.ser:
                    return self.reply(404, {"error": f"board {parts[1]} not open"})
                b.ser.write(data.rstrip(b"\r\n") + b"\n")
                return self.reply(200, {"ok": True})
            if parts[0] in ("flash", "reset"):
                kind = q.get("kind", ["esp32"])[0]
                if not b and not (parts[0] == "flash" and kind == "rp2040" and hub.uf2_drive()):
                    return self.reply(404, {"error": f"no board {parts[1]}"})
                if not hub.global_flash.acquire(timeout=120):
                    return self.reply(409, {"error": "another flash is running"})
                try:
                    if parts[0] == "reset":
                        ok, out = hub.reset_esp(b)
                    elif kind == "rp2040":
                        ok, out = hub.flash_uf2(b, data)
                    else:
                        ok, out = hub.flash_esp(b, data)
                finally:
                    hub.global_flash.release()
                return self.reply(200 if ok else 500, {"ok": ok, "output": out})
            self.reply(404, {"error": "unknown path"})

    return H


def find_esptool():
    pats = [os.path.expandvars(r"%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esptool_py\*\esptool.exe"),
            os.path.expanduser("~/.arduino15/packages/esp32/tools/esptool_py/*/esptool")]
    for pat in pats:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return shutil.which("esptool") or shutil.which("esptool.py") or "esptool"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--token", required=True)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--esptool", default=None)
    a = ap.parse_args()
    if sys.stdout is None or sys.stderr is None:  # pythonw: no console, log to a file instead
        f = open(os.path.join(HERE, "board_hub.out"), "a", encoding="utf-8", buffering=1)
        sys.stdout = sys.stderr = f
    hub = Hub(a.esptool or find_esptool())
    threading.Thread(target=hub.scan_forever, daemon=True).start()
    print(f"board hub on {a.bind}:{a.port}, esptool={hub.esptool_cmd}", flush=True)
    ThreadingHTTPServer((a.bind, a.port), make_handler(hub, a.token)).serve_forever()


if __name__ == "__main__":
    main()
