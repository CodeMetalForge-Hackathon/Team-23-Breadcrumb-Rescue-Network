"""Rescue Net wire protocol: one compact JSON object per line, < 200 bytes.

Shared by the spotter (Pi) and the command post (laptop). The ESP32 firmware
depends on this format: do not change keys without telling the firmware side.
"""
import json
import logging
import secrets

MAX_BYTES = 200          # whole line including the trailing "\n" (LoRa packet limit)
TYPES = ("PERSON", "HEAT", "PHONE", "SOS", "HAZARD", "HB")

log = logging.getLogger("protocol")


def new_id():
    """Random 4-hex-char message ID, used by every node for de-duplication."""
    return secrets.token_hex(2)


def make_msg(t, src, ttl=4, **fields):
    """Build a message dict: id, src, ttl, rt, t, then fields in the order given. None fields are dropped."""
    msg = {"id": new_id(), "src": src, "ttl": ttl, "rt": [], "t": t}
    for key, value in fields.items():
        if value is not None:
            msg[key] = value
    return msg


def encode_line(msg):
    """Serialize to compact UTF-8 JSON + newline. Raises ValueError if too big."""
    line = (json.dumps(msg, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    if len(line) > MAX_BYTES:
        log.error("message too big (%d > %d bytes), not sent: %r", len(line), MAX_BYTES, line[:80])
        raise ValueError(f"message is {len(line)} bytes, limit is {MAX_BYTES}")
    return line


def extract_message(obj):
    """Parsed firmware line -> Rescue Net message dict, or None.
    Accepts a bare message, or a wrapper like {"lora_rx": {...message...}, "rssi": -80}
    (the rssi is kept as "_rssi"). Status lines such as {"sent": "a3f1", "len": 59} are not messages."""
    if not isinstance(obj, dict):
        return None
    if "t" in obj and "src" in obj:
        return obj
    for value in obj.values():
        if isinstance(value, dict) and "t" in value and "src" in value:
            msg = dict(value)
            if isinstance(obj.get("rssi"), (int, float)):
                msg["_rssi"] = obj["rssi"]
            return msg
    return None


def parse_line(line):
    """Parse one serial line. Returns a dict, or None for anything that is not a JSON object."""
    if line is None:
        return None
    if isinstance(line, bytes):
        line = line.decode("utf-8", errors="replace")
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        obj = json.loads(line)
    except ValueError:
        return None
    return obj if isinstance(obj, dict) else None
