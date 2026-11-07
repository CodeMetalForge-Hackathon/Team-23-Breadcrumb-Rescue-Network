"""Append-only daily JSONL logs in <repo>/logs/<prefix>-YYYYMMDD.jsonl."""
import json
import time
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent.parent / "logs"


def append(prefix, obj):
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"{prefix}-{time.strftime('%Y%m%d')}.jsonl"
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n")
