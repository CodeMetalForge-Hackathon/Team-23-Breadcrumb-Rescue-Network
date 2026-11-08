# Technical slides: reference material

Slide-by-slide content for building the presentation. Every number below is from the [README status table](../README.md#status-what-is-verified-on-hardware); "verified" means measured on our hardware on Nov 7-8, 2026, and "implemented" means running but not yet live-tested. Images are in [`docs/media/`](media/).

Images:

| File | What it shows |
|---|---|
| `media/architecture.png` | System diagram: spotter, SP1, B2, B1, BASE, command post, survivor phone; alert chain and bulk link |
| `media/dashboard_live_full.png` | Live command-post dashboard (Nov 8, 08:13 EST): floor plan, relay chain, alerts, live panes |
| `media/dashboard_floorplan_controls.png` | Floor-plan panel with its controls (Fit, Labels, Raw scan, Signal, Mark relay, Pause map) and the People list |
| `media/dashboard_relay_chain_alerts.png` | Relay chain health (SP1, B2, B1, BASE) and alert cards (PERSON, PHONE) with "Mark handled" |
| `media/dashboard_live_panes.png` | Camera, thermal and LiDAR panes streamed over the bulk link, with the link rate and level |
| `media/floorplan_walk3_43m.png` | Floor plan from the 43 m walk: wall segments, walked path, BASE, "place relay N" suggestions |
| `media/portal_sos_form.png` | Survivor SOS page on the relay Wi-Fi `RESCUE-TEAM-NEARBY` (rendered from the B1 firmware) |
| `media/portal_sos_sent_waiting.png` | Page after sending: waits for the command post, then switches to "Received by rescuers" |
| `media/hw_spotter_pi5_ai_hat.jpg` | Spotter core: Raspberry Pi 5 with the AI HAT+ (Hailo-8, 26 TOPS) |
| `media/hw_relays_b1_b2_on_lipo.jpg` | Relays B1 and B2 on LiPo batteries with an RFM95W LoRa radio, showing chain status |

---

## 1. Problem

- Collapsed or smoke-filled building: no GPS, no cell service, no Wi-Fi, no up-to-date map.
- Rescuers need three things back at the command post: **who** is inside, **where**, and **what the building looks like**.

## 2. System overview (image: `architecture.png`)

- **Handheld spotter**: Raspberry Pi 5 16 GB + AI HAT+ (Hailo-8), Camera Module 3, USB thermal camera (640x512), Slamtec RPLIDAR C1, MMA7660 accelerometer, USB power bank.
- **Breadcrumb relays**: Adafruit Feather ESP32-S3 Reverse TFT boards on LiPo cells, dropped as the responder walks.
  - SP1: Feather RP2040 RFM95, the spotter's LoRa bridge on USB.
  - B2: gateway relay, LoRa receiver (RFM95W), BLE phone listener, hidden `RESCUE-LINK` AP for bulk data.
  - B1: relay and survivor captive portal `RESCUE-TEAM-NEARBY`.
  - BASE: feeds the command-post laptop over USB.
- **Command post**: any laptop running the Flask dashboard; no internet, no cloud.

## 3. Two radio links

| | Alerts | Bulk data |
|---|---|---|
| Content | "person found", "SOS", phone counts, node marks: short-key JSON, under 200 bytes | camera JPEG, thermal JPEG, LiDAR scan (731 B), people/link metadata |
| Path | Pi → USB → SP1 → LoRa 915 MHz → B2 → ESP-NOW flood → B1 → BASE | Pi → UDP over B2's hidden AP → ESP-NOW 12 Mbps unicast → B1 → BASE |
| Speed | under 1 s end to end (verified Nov 7) | about 420 kbps (verified Nov 8) |
| Resilience | every relay floods with de-duplication, hop limit and random back-off; routes around a silent relay (verified Nov 7) | sheds pictures first when the Wi-Fi signal weakens (implemented, unit-tested) |

Bulk link levels, picked from the Pi's Wi-Fi RSSI to B2:

| Level | RSSI | LiDAR | Meta | Pictures |
|---|---|---|---|---|
| good | above -67 dBm | 10 Hz | 4 Hz | 2 fps at 320 px |
| fair | -67 to -77 dBm | 10 Hz | 2 Hz | 1 fps at 240 px |
| poor | below -77 dBm | 5 Hz | 2 Hz | one every 4 s at 160 px |

## 4. Finding people (OBJ.01, OBJ.02)

- YOLOv8-pose on the Hailo-8, run on the RGB camera **and** on the thermal camera, so detection works in the dark (verified Nov 7).
- Posture from keypoints: lying vs upright, 4-of-6 frame debounce. A person on the ground is ranked first.
- **Person-down latency: 1.73 s** from the start of the fall to the dashboard card; 0.73 s from the first "down" frame, of which 0.59 s is radio transport (measured Nov 8, 06:15 EST).
- Position: bearing from the box and camera yaw/HFOV, range from the nearest LiDAR returns, cross-checked against range from box size.
- Fewer false alerts (implemented, unit-tested): no re-alert for the same person within 15° / 60 s; "down" needs a visible torso and thermal confirmation; in a lit scene a thermal-only person inside the RGB view is treated as a reflection. Work on false alerts is ongoing.

## 5. Mapping the building (OBJ.03) (image: `floorplan_walk3_43m.png`)

- Handheld 2D scan-matching SLAM in NumPy only; scans taken while the rig tilts more than 10° are rejected using the accelerometer.
- **Verified Nov 8 (walk 3, live over the relays)**: about 43 m walked, ends 0.46 m from the start, 2628 of 3044 scans accepted, 34 wall segments, 3 people placed by bearing and range.
- Room-style plan: wall segments extracted from the occupancy grid in about 70 ms (implemented, unit-tested).
- Each relay is registered on the map by pressing its D2 button next to the spotter (verified Nov 8, 06:17 EST).
- Walked path coloured by measured Wi-Fi signal (implemented).
- 3D point cloud view with floor and ceiling returns hidden (implemented, unit-tested).

## 6. Survivors with a phone (OBJ.01) (images: `portal_sos_form.png`, `portal_sos_sent_waiting.png`)

- Any phone near B1 sees the open Wi-Fi `RESCUE-TEAM-NEARBY`; a captive-portal page pops up. No app, no internet.
- One-tap SOS, plus how many people, injuries, urgent conditions, hazards (smoke, fire, water, gas, unstable ceiling), and a free-text location.
- The command post replies and the phone switches to "Received by rescuers" (verified Nov 7 with an iPhone).
- Relays count nearby phones over BLE; the plan shows a halo at each marked relay. A zone, not a position (implemented).
- SOS card says "about N m from B1" from the phone's Wi-Fi signal, d = 10^((-40 - RSSI)/25) m, ±6 dB band (implemented; live iPhone test pending).

## 7. Staying up when things fail (OBJ.04)

- Alert flood heals around a silent relay (verified Nov 7).
- SP1 reboots itself after 40 s of USB silence, backed by a hardware watchdog; the laptop hub reopens a stalled serial port (verified Nov 7/8).
- The Pi re-joins `RESCUE-LINK` about 12 s after B2 reboots (verified Nov 8).
- Bulk link shows LINK DOWN after 5 s without spotter data (implemented).
- "Pause map" while the rig is set down; mapping resumes at the same pose (implemented, unit-tested).

## 8. Where to drop the next relay (OBJ.04)

- The planner estimates link margin from distance and wall crossings along the walked path and marks "place relay N" spots on the plan (implemented).
- The radio model is not calibrated yet; fitting it from RSSI measured along the walk is in progress.

## 9. What we tried and dropped

- **Monocular depth** (Depth Anything V2 Small, scaled by the LiDAR) on 25 live sightings: read a median 50% too far, median error 1.0 m, against 0.25 m from box size. Dropped.
- **Nodding the rig for height** in the 3D view: distorted the point cloud too much. Dropped Nov 8; only a guard that discards scans tilted past 30° remains.
- **Tilted LiDAR**: 6° off level made floor hits look like walls; re-mounted to about 3°.
- **Onboard speaker**: too noisy, shelved.

## 10. Deployability

- All parts off the shelf: Raspberry Pi 5 + AI HAT+, Adafruit Feather boards, RPLIDAR C1, USB thermal camera.
- Relays on LiPo cells, spotter on a USB power bank.
- No cloud, no app: our own radios end to end; survivors use a normal phone browser.
- Tens of dollars per relay, cheap enough to leave behind.

## 11. Next steps

- Phone trilateration: B1, B2, BASE and the walking spotter report per-phone BLE signal; weighted least-squares fix from the relay positions marked with D2. Expected 3-5 m indoors.
- UWB ranging between relays for exact relay positions.
- Calibrated radio model from measured links, for relay placement and SOS distance.
- Rugged, sealed enclosures.

## Software stack

- Spotter (Python on the Pi): picamera2 + HailoRT (`yolov8s_pose_h8.hef` from the Hailo Model Zoo), OpenCV, NumPy, pyserial, smbus2.
- Firmware (Arduino): Arduino-ESP32 (ESP-NOW, Wi-Fi, BLE), RadioHead RH_RF95, ArduinoJson, Adafruit GFX/ST7789.
- Command post: Flask dashboard on :5050, board hub that owns the serial ports.
- Tests: pytest suite that runs without hardware (`cd rescue-net && python -m pytest -q`).
