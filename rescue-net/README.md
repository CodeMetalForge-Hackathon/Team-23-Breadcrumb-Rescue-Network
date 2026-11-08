# Rescue Net: spotter + command post
Setup: laptop `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`; Pi `python3 -m venv --system-site-packages .venv` (see requirements.txt).
Pi checks (M0): `bash spotter/envcheck.sh 2>&1 | tee envcheck.txt`
Spotter: `python -m spotter.main [--mock] [--mock-thermal] [--out serial|stdout] [--port /dev/ttyUSB0] [--no-debugview]`. Debug view at `http://<pi>:8080/`; tune `spotter/config.json` live.
Dashboard: `python -m dashboard.app [--port /dev/cu.usbserial-XXXX | COM3]` (auto-detects the ESP32), then open http://localhost:5050 (not 5000: macOS AirPlay uses it).
No hardware: `python -m spotter.main --mock --out stdout | python -m dashboard.app --stdin`, or `python -m dashboard.app --mock [--speed 5]`.
Demo backup: `python -m dashboard.app --replay logs/messages-YYYYMMDD.jsonl`.
Tests: `python -m pytest -q`. Logs: `logs/spotter-*.jsonl` (sent), `logs/messages-*.jsonl` (received).
Wire format and rules: one JSON object per line, 115200 baud, < 200 bytes, see `common/protocol.py`.
