"""Where spotter messages go: the ESP32 over USB serial, or stdout for testing.
Every message actually sent is logged to logs/spotter-YYYYMMDD.jsonl."""
import logging
import queue
import sys
import threading
import time

import serial

from common import jsonlog, serialport
from common.protocol import encode_line, parse_line

log = logging.getLogger("sink")


def _record(msg, line):
    jsonlog.append("spotter", {"tx": round(time.time(), 3), "msg": msg})
    log.info("SENT %s", line.decode("utf-8").rstrip())


class StdoutSink:
    def send(self, msg):
        line = encode_line(msg)
        sys.stdout.buffer.write(line)
        sys.stdout.buffer.flush()
        _record(msg, line)

    def close(self):
        pass


class SerialSink:
    """send() only queues; a writer thread owns the port, so the detection loop never blocks.
    Reconnects automatically if the board is unplugged. The board's {"sent": id} confirmations are logged;
    anything else it prints (e.g. packets it hears) is logged with -v."""

    def __init__(self, port=None, baud=115200):
        self.port = port
        self.baud = baud
        self.q = queue.Queue(maxsize=50)
        self._stop = threading.Event()
        threading.Thread(target=self._run, name="SerialSink", daemon=True).start()

    def send(self, msg):
        line = encode_line(msg)          # raises ValueError if too big, before it is queued
        try:
            self.q.put_nowait((msg, line))
        except queue.Full:
            log.error("serial queue full, dropping %s", line.decode("utf-8").rstrip())

    def close(self):
        self._stop.set()

    def _connect(self):
        device = self.port or serialport.find_port()
        if not device:
            return None
        ser = serialport.open_port(device, self.baud, timeout=0)
        log.info("serial connected: %s @ %d", device, self.baud)
        return ser

    def _board_output(self, data):
        self._buf += data.decode("utf-8", errors="replace")
        *lines, self._buf = self._buf.split("\n")
        for raw in lines:
            obj = parse_line(raw)
            if obj is not None and "sent" in obj:
                log.info("radio sent %s (%s bytes)", obj["sent"], obj.get("len"))
            elif raw.strip():
                log.debug("board> %s", raw.strip())

    def _run(self):
        ser, pending, warned = None, None, False
        self._buf = ""
        while not self._stop.is_set():
            if ser is None:
                try:
                    ser = self._connect()
                except (serial.SerialException, OSError) as e:
                    ser = None
                    log.warning("serial open failed: %s", e)
                if ser is None:
                    if not warned:
                        log.warning("no LoRa board serial port yet, retrying every 2 s (messages are queued)")
                        warned = True
                    time.sleep(2)
                    continue
                warned = False
            try:
                if pending is None:
                    try:
                        pending = self.q.get(timeout=0.2)
                    except queue.Empty:
                        pass
                if pending is not None:
                    ser.write(pending[1])
                    ser.flush()
                    _record(*pending)
                    pending = None
                if ser.in_waiting:
                    self._board_output(ser.read(ser.in_waiting))
            except (serial.SerialException, OSError) as e:
                log.warning("serial lost (%s), reconnecting", e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
