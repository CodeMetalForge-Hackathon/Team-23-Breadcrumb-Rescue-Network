# Rescue Net: spotter, command post, mapping

The Python half of the Breadcrumb Rescue Network. See the [top-level README](../README.md) for the overview and the hardware. Shivansh's original version of this directory is tagged `shivansh-original`.

## Data paths

```
ALERTS (small JSON, < 200 B, every relay floods them)
  spotter.main --USB serial--> SP1 (LoRa 915 MHz) --> B2 --ESP-NOW--> B1 --ESP-NOW--> BASE --USB--> board hub --> dashboard

BULK (camera / thermal / LiDAR / meta, about 420 kbps)
  spotter.bulklink --UDP over B2's hidden AP RESCUE-LINK (192.168.4.1:5005)--> B2 --ESP-NOW unicast--> B1 --> BASE
  BASE prints "$B <kind> <seq> <idx> <cnt> <base64>" lines; dashboard.remote reassembles them
```

Bulk object kinds:

| Kind | Content |
|---|---|
| 1 | camera JPEG, with person boxes and skeletons drawn in |
| 2 | thermal JPEG |
| 3 | LiDAR: 360 × uint16 mm, one bin per degree, then `t` (float64) and the accelerometer (3 × int8), 731 B in total |
| 4 | meta JSON: tracked people with bearing, range and posture, plus link level, RSSI and tilt |

Link levels, picked automatically from the Pi's Wi-Fi RSSI to B2:

| Level | RSSI | LiDAR | Meta | Pictures |
|---|---|---|---|---|
| 0 | above -67 dBm | 10 Hz | 4 Hz | 2 fps at 320 px |
| 1 | -67 to -77 dBm | 10 Hz | 2 Hz | 1 fps at 240 px |
| 2 | below -77 dBm | 5 Hz | 2 Hz | one every 4 s at 160 px |

The map and the people keep flowing at every level.

## Modules

| Path | What it does |
|---|---|
| `spotter/main.py` | Entry point on the Pi. It wires the camera detector, thermal, LiDAR, IMU, tracker, LoRa sink, bulk link and debug view (:8080) together. |
| `spotter/detector.py` | YOLOv8-pose on the Hailo-8 for the Camera Module 3, or a mock |
| `spotter/ircam.py`, `thermal_detect.py` | USB UVC thermal camera, with pose run on thermal frames. Gives thermal-only PERSON alerts (`s:"T"`). |
| `spotter/posture.py`, `debounce.py`, `fusion.py` | Lying/upright posture from keypoints (`ps` D/U, 4-of-6 debounce); camera/thermal fusion |
| `spotter/lidar.py` | RPLIDAR C1 reader. Re-probes its port if the LiDAR is unplugged. |
| `spotter/locate.py`, `tracker.py` | Bearing from the box and the camera yaw/HFOV, and range from the nearest LiDAR returns. Per-person tracks (`k`); one alert per track. |
| `spotter/imu.py` | MMA7660 accelerometer on I2C-1, address 0x4C |
| `spotter/sink.py` | Serial link to SP1. Reconnects when SP1 reboots. `main.py` sends a heartbeat every 15 s, which SP1's USB self-heal watches for. |
| `spotter/bulklink.py` | Bulk link scheduler and auto-degrade |
| `spotter/calib.py` | Records and fits the camera-to-LiDAR yaw and HFOV |
| `dashboard/app.py` | Flask command post on :5050: alert list, relay chain, SOS reply (ACK), floor plan, live panes |
| `dashboard/remote.py` | Reassembles bulk objects from the `$B` lines |
| `dashboard/mapview.py` | Floor plan state. It feeds LiDAR into SLAM and places people from meta or from the LoRa `b`/`r` fields. Relay nodes come from D2 MARK presses or drags and are saved to `logs/map_nodes.json`. Also renders the map PNG and the relay suggestions. |
| `mapping/slam.py` | 2D scan-matching SLAM in NumPy only, with an accelerometer tilt gate. See [mapping/README.md](mapping/README.md). |
| `mapping/relay.py` | Relay placement heuristic: link margin from distance and wall crossings along the walked path |
| `mapping/replay.py` | Offline: builds a floor plan PNG from a recorded walk |
| `common/protocol.py` | Wire format: short-key JSON, one object per line, 115200 baud, under 200 bytes |

## Setup

- **Laptop:** `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`
- **Pi:**
  1. `python3 -m venv --system-site-packages .venv`, so that picamera2, Hailo and OpenCV come from apt.
  2. Install the remaining packages from `requirements.txt`, plus `smbus2`.
  3. Enable I2C: `dtparam=i2c_arm=on`.
  4. Check the environment: `bash spotter/envcheck.sh 2>&1 | tee envcheck.txt`.

## Running

- **Spotter:** `python -m spotter.main [--mock] [--mock-thermal] [--no-thermal] [--no-bulk] [--out serial|stdout] [--port /dev/serial/by-id/...] [--no-debugview]`
  - The debug view is at `http://<pi>:8080/`. Tune `spotter/config.json` live.
- **Dashboard:** `python -m dashboard.app [--port COM4 | /dev/ttyACM0]`, or with `--hub http://127.0.0.1:8765` when `tools/board_hub.py` owns the port.
  - Open http://localhost:5050. It is not on 5000, because macOS AirPlay uses that port.
- **No hardware:**
  - `python -m spotter.main --mock --out stdout | python -m dashboard.app --stdin`
  - or `python -m dashboard.app --mock [--speed 5]`
- **Demo backup:** `python -m dashboard.app --replay logs/messages-YYYYMMDD.jsonl`
- **Map from a recorded walk:** `python -m mapping.replay walk.jsonl.gz out.png`. Record a walk with `tools/record_walk.py` on the Pi while the spotter service is stopped.
- **Tests:** `python -m pytest -q`. They run without hardware.
- **Logs:** `logs/spotter-*.jsonl` holds what was sent, `logs/messages-*.jsonl` what was received.

## Known limits

- B2 is the spotter's only gateway: only B2 has the LoRa radio and the `RESCUE-LINK` AP, so if B2 fails the spotter is cut off, and B1 or BASE cannot take over.
- Power: the Pi rebooted several times on a weak supply, so the spotter needs the official Pi PSU or a 27 W USB-C PD power bank. Detection runs at 5 fps and the camera at 10 fps, with Bluetooth off, to keep the load down (spotter CPU about 40%, Pi about 3.1 W, no under-voltage).
- The Pi 5's onboard Wi-Fi has no monitor mode, so the spotter cannot listen for an SOS phone.
- The LiDAR is mounted about 3° off level (re-mounted from 6°), and that tilt is not corrected geometrically. Floor or ceiling returns can show up as walls. Scans taken while the rig is tilted more than 10° are rejected.
- SLAM has no loop closure. Long featureless corridors can make it drift.
- Relay suggestions come from an uncalibrated radio heuristic, not from measured links.
- The thermal camera gives a relative (AGC) image, not absolute temperature.
