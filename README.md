# Breadcrumb Rescue Network

**Team 23, Code Metal Forge Hackathon 2026 ("Operation Lost and Found").** Boston, Nov 6-8, 2026.

A responder walks into a building that has no GPS, cell service or Wi-Fi. They carry a handheld spotter: a Raspberry Pi 5 with an AI accelerator, an RGB camera, a thermal camera and a 2D LiDAR. As they go, they drop small radio relays ("breadcrumbs"). The spotter finds people, in the dark too, and works out where they are. It also draws a floor plan of the building. Everything it learns goes back to a command-post laptop over the team's own relay chain, with no outside network.

- **Alerts** ("person found", "SOS") travel over LoRa and an ESP-NOW flood mesh. Every relay forwards them, so the chain heals itself when one relay goes quiet.
- **Bulk data** (camera, thermal, LiDAR scans, people positions) travels over a separate fast link through the same relays. When the radio signal gets weak, it sheds the pictures first.
- **Survivors with a phone** can join the `RESCUE-TEAM-NEARBY` Wi-Fi on a relay and send an SOS from a captive-portal page. They see "Received by rescuers" when the command post acknowledges it.

![Architecture](docs/architecture.png)

## How it maps to the challenge

| Objective | What we built |
|---|---|
| OBJ.01 Locate isolated people | Each detected person gets a bearing (from the camera) and a range (from the LiDAR). The spotter's SLAM pose turns these into a point on the floor plan. Phones near a relay are counted over BLE. The captive portal gives survivors a way to report in. |
| OBJ.02 Detect life and hazards | YOLOv8-pose on the Hailo-8 runs on the RGB camera and also on the thermal camera, so it still finds people in low light. Posture tells lying apart from upright, and the dashboard ranks a person on the ground first. Survivors can report hazards through the SOS form. |
| OBJ.03 Map changing conditions | The handheld LiDAR runs 2D SLAM, with an accelerometer to reject scans taken while the rig is tilted. The command post shows a live floor plan with the walked path, the people and the relay positions. |
| OBJ.04 Deliver critical information | There are two links. LoRa plus the ESP-NOW flood mesh carries small alerts and needs no infrastructure. The bulk link (about 420 kbps) adapts to signal quality. The map suggests where to drop the next relay. |

## Mentor feedback we designed for

1. **Low light.** Pose detection also runs on the thermal camera. A thermal-only detection still raises a PERSON alert, marked `s:"T"`.
2. **Environment modelling.** The handheld 2D SLAM builds a floor plan on the dashboard, drawn as clean wall segments. A 3D point cloud view adds height: pause the map, then nod the rig.
3. **Node-failure resilience.** Alerts flood through every relay with de-duplication and a hop limit, so they route around a relay that goes silent. The spotter's LoRa bridge (SP1) reboots itself when its USB link stalls, and a hardware watchdog backs that up. The laptop hub reopens a serial port that goes silent.
4. **Best relay placement.** The planner estimates link margin from the walked path, the distance and the walls on the map. It suggests where the next relay should go, labelled "place relay N" on the map. Operators register a real relay by pressing its D2 button while standing next to it, or by dragging it on the map.

## Status (what is verified on hardware)

| Capability | Status |
|---|---|
| LoRa SP1 → B2 → ESP-NOW B1 → BASE alert chain, under 1 s end to end | verified 2026-11-07 |
| Captive-portal SOS from an iPhone, with an ACK back to the phone | verified 2026-11-07 |
| Alert flood healing around a silent relay | verified 2026-11-07 |
| SP1 USB self-heal (reboot after 40 s of silence) and hub serial reopen | verified 2026-11-07/08 |
| YOLOv8-pose on Hailo-8 (RGB) and on the USB thermal camera | verified 2026-11-07 |
| Bulk link Pi → B2 → B1 → BASE (about 420 kbps), with camera, thermal and LiDAR panes on the dashboard | verified 2026-11-08 |
| Auto-degrade of the bulk link on weak Wi-Fi signal | implemented, unit-tested (`tests/test_bulklink.py`) |
| Person-down alert latency: 1.73 s from the start of the fall to the dashboard card (0.73 s from the first "down" frame, of which 0.59 s is transport) | measured 2026-11-08 06:15 EST |
| Node button D2 MARK registers B1 / B2 / BASE at the spotter's SLAM pose | verified 2026-11-08 06:17 EST (MARK from B1 and B2 arrived and placed) |
| "Pause map" while the rig is set down; mapping resumes at the same pose | implemented, unit-tested (`tests/test_mapping_slam.py`) |
| Bulk-link status shows LINK DOWN after 5 s without spotter data | implemented |
| Pi Wi-Fi watchdog re-joins RESCUE-LINK after B2 reboots | implemented, unit-tested (`tests/test_linkwatch.py`); live test pending |
| Handheld 2D SLAM floor plan with walked path and people on the map | verified 2026-11-08 05:46-05:53 EST (walk3, live over the bulk link: about 43 m walked, ends 0.46 m from the start, 2628 of 3044 scans accepted, 34 wall segments, 3 people placed by bearing and range) |
| Walked path coloured by measured Wi-Fi signal to B2 (good at -67 dBm and above, fair down to -77, poor below) | implemented, deployed 2026-11-08 06:27 EST |
| Room-style plan: wall segments extracted from the SLAM grid (denoised, about 70 ms) | implemented, unit-tested (`tests/test_floorplan.py`) |
| Suggested relay spots on the floor plan, labelled "place relay N" | implemented; calibrating the radio model from measured RSSI along the walk is in progress |
| 3D point cloud view, with estimated floor and ceiling returns hidden by default (Floor/ceiling toggle) | implemented, unit-tested (`tests/test_cloud_filter.py`, `tests/test_cloud_filter_replay.py`) |
| Nodding the rig while the map is paused adds height to the 3D cloud; scans tilted past 30° are dropped | implemented; live test pending |

## Hardware

| Role | Hardware |
|---|---|
| Spotter (handheld) | Raspberry Pi 5 16 GB with the Raspberry Pi AI HAT+ (Hailo-8, 26 TOPS); Camera Module 3 (IMX708); USB thermal camera (640x512, UVC); Slamtec RPLIDAR C1 (USB, CP2102N); Grove MMA7660 accelerometer (I2C); USB power bank |
| SP1, the spotter's LoRa bridge | Adafruit Feather RP2040 RFM95 (#5714), 915 MHz, 5 dBm, on the Pi's USB |
| B2, gateway relay | Adafruit Feather ESP32-S3 Reverse TFT (#5691) with an RFM95W LoRa breakout (#3072). It receives LoRa, listens for BLE phones, and runs the hidden `RESCUE-LINK` AP that the Pi uses for bulk data. |
| B1, relay and survivor portal | Feather ESP32-S3 Reverse TFT. It runs the `RESCUE-TEAM-NEARBY` captive portal. |
| BASE, command post | Feather ESP32-S3 Reverse TFT on USB to the laptop |
| Command post | Any laptop with Python 3, running the dashboard at http://localhost:5050 |

## Repository layout

```
rescue-net/            Python: spotter (Pi), command-post dashboard (laptop), mapping, tests
  spotter/             detection (Hailo YOLOv8-pose), thermal, LiDAR, IMU, tracking, LoRa sink, bulk link
  dashboard/           Flask command post: alerts, relay chain, floor plan, live panes
  mapping/             2D SLAM (numpy only), relay planner, offline replay
  common/              wire protocol, serial helpers, JSON logs
  tests/               pytest suite (runs without hardware)
firmware/
  lora_bridge/         SP1: USB JSON → LoRa (Feather RP2040 RFM95)
  breadcrumb_espnow/   B2 / B1 / BASE: ESP-NOW flood relay, LoRa gateway, BLE, captive portal, bulk link
tools/                 laptop board hub (serial ↔ HTTP, starts the dashboard), walk recorder
docs/                  architecture diagram, demo script, write-up
notes/, photos/        build log and photos from the weekend
```

## Setup and running the demo

Details are in [rescue-net/README.md](rescue-net/README.md). In short:

1. **Flash the boards** with the Arduino IDE or `arduino-cli`, using the ESP32 core and the libraries ArduinoJson, RadioHead and Adafruit ST7789/GFX.
   - SP1: `firmware/lora_bridge/lora_bridge.ino`, built for the Feather RP2040 RFM95.
   - B2, B1 and BASE: `firmware/breadcrumb_espnow/breadcrumb_espnow.ino`, built with `-DNODE_ID=\"B2\"`, `\"B1\"` or `\"BASE\"`.
2. **Set up the Pi:**
   ```
   cd rescue-net
   python3 -m venv --system-site-packages .venv
   .venv/bin/pip install -r requirements.txt
   ```
   Then `python -m spotter.main --port /dev/serial/by-id/<SP1>`. In the demo it runs as the systemd service `rescue-spotter`. The Pi joins B2's hidden `RESCUE-LINK` AP for the bulk link.
3. **Start the command post** on the laptop:
   ```
   cd rescue-net
   pip install -r requirements.txt
   python -m dashboard.app --port <BASE serial port>
   ```
   Then open http://localhost:5050. In the demo, `tools/board_hub.py` owns the serial ports and starts the dashboard.
4. **No hardware?** Run `python -m dashboard.app --mock`, or replay a log with `--replay logs/messages-YYYYMMDD.jsonl`. `python -m mapping.replay <walk.jsonl.gz> out.png` builds a floor plan from a recorded walk.
5. **Tests:** `cd rescue-net && python -m pytest -q`.

## Team

- Xiaofan Liu (UMass Amherst): radio firmware, bulk link, command post, mapping integration
- Shivansh Chhawri (UMass Amherst): spotter (on-device detection), original system plan, first dashboard
- Danica Catoni (remote): breadcrumb relay concept and roadmap
- Ziwen Xie (UMass Amherst, remote support)

Xiaofan also used AI coding assistants (Claude Code, Codex) throughout the build.

## Credits

All code here was written during the event. Shivansh's original spotter and dashboard are imported unmodified at commit `04ff889`, tagged `shivansh-original`; later commits build on them.

We used these existing libraries and models:

- Ultralytics YOLOv8-pose (AGPL-3.0), as the precompiled `yolov8s_pose_h8.hef` from the Hailo Model Zoo, run with HailoRT through `picamera2.devices.Hailo`
- `picamera2` / libcamera, OpenCV, NumPy, Flask, pyserial, smbus2
- Slamtec RPLIDAR C1 serial protocol
- Arduino-ESP32 / ESP-IDF (ESP-NOW, WiFi, BLE), ArduinoJson, RadioHead (RH_RF95), Adafruit GFX and ST7789

## License

Apache License 2.0 (see `LICENSE`).
