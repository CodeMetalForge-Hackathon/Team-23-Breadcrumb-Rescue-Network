# Breadcrumb Rescue Network: project story

## Inspiration

When a building collapses or a wildfire takes out the towers, rescuers lose the tools they rely on: GPS, cell service and Wi-Fi. The challenge asked how to locate, identify and rescue people in exactly those conditions. Cave and mine rescue teams already do the simplest version of this: they leave a trail of markers behind them. We wanted a trail that talks: cheap radio relays dropped along the way, carrying what a handheld sensor sees back to the command post.

## What it does

A responder carries a handheld **spotter**: a Raspberry Pi 5 with an AI accelerator, an RGB camera, a thermal camera, a 2D LiDAR and an accelerometer. As they walk, they drop **breadcrumb relays**.

- **Finds people, in the dark too.** Pose detection runs on the Hailo-8 on both the camera and the thermal image, and someone lying on the floor is flagged first. The camera gives the bearing and the LiDAR the range, so each person is placed on the map.
- **Maps the building.** Handheld 2D SLAM from the LiDAR builds a floor plan at the command post, with the responder's path. Each relay is registered on that map with one button press.
- **Gets the information out without infrastructure.** Alerts are tiny JSON messages sent over LoRa and an ESP-NOW flood mesh. When a relay dies, the mesh routes around it within seconds. A second, faster link (about 420 kbps over ESP-NOW) carries the live camera, thermal, LiDAR and map data. When the signal weakens, it drops the pictures first, so the map and the people keep flowing.
- **Lets survivors call in.** Any phone near a relay sees the `RESCUE-TEAM-NEARBY` Wi-Fi, and a page pops up for sending an SOS (how many people, injuries, hazards). When the command post replies, the phone shows "Received by rescuers". Relays also count nearby phones over BLE. After an SOS, the other relays listen for that phone for 5 s, and the command post draws a search area among the relays that heard it (listening verified with an iPhone).
- **Plans the next relay.** From the map's walls and the path walked so far, the dashboard suggests where to drop the next breadcrumb before the link runs out. The relay in hand watches its own signal back to the chain and, when it falls below -70 dBm, shows DROP HERE on its screen, beeps the spotter's speaker and puts a banner on the dashboard (bench-verified; not yet tried on a real walk).
- **Survives failures, and shows it.** Any relay except the gateway B2 can be switched off by holding its D1 button for 1 s; the chain heals around it in about 8 s. A relay whose radio or main loop stalls restarts itself in about 3 s and keeps its settings. Every relay reports its battery level. On a weak signal the operator can drop to LoRa only: alerts keep coming, pictures and the map pause.

## How we built it

- **Spotter (Python on the Pi):**
  - YOLOv8-pose compiled for the Hailo-8, run on the camera and on letterboxed thermal frames;
  - a keypoint posture classifier with debouncing;
  - per-person tracks fused with RPLIDAR C1 ranges;
  - a bulk-link scheduler that adapts to the Wi-Fi RSSI.
- **Radios (Arduino):**
  - a Feather RP2040 RFM95 bridges the Pi's USB to LoRa;
  - three Feather ESP32-S3 boards form the relay chain. B2 is the LoRa gateway and also runs a hidden access point that the Pi streams bulk data into. B1 hosts the captive portal. BASE feeds the laptop.
  - Relays flood alerts with de-duplication, a hop limit and random back-off.
- **Command post (Python/Flask):**
  - a board hub owns the serial ports;
  - the dashboard shows the alerts, the health of the relay chain, the SOS messages, the floor plan and the live panes.
- **Mapping:** scan-matching SLAM written in NumPy only, with an accelerometer gate that rejects scans taken while the rig is tilted. A relay planner estimates link margin from distance and wall crossings.

## Challenges we ran into

- **Only two LoRa radios.** That forced the split: LoRa for the first hop out of the spotter, ESP-NOW between relays.
- **USB links freeze.** The spotter's USB serial to its LoRa bridge went dead twice. The bridge now reboots itself after 40 s of silence, under a hardware watchdog, and the laptop hub reopens stalled serial ports.
- **Hardware.** The AI HAT's stacking header had been mounted upside down, which cut off every GPIO pin until we found it. The first speaker setup (PWM audio on a GPIO pin) was too noisy for speech, so placement prompts are distinct beep patterns from the Pi's GPIO speaker, plus DROP HERE on the relay screen and a dashboard banner.
- **A handheld, tilted LiDAR is a hard SLAM input.** It sat 6° off level (we re-mounted it to about 3°), and people walk unevenly. We gate scans on the accelerometer and reject bad matches rather than corrupt the map.
- **Monocular depth did not beat simple geometry.** We tried Depth Anything V2 Small, scaled by the LiDAR, to range people. On 25 live observations it read a median 50% too far (median error 1.0 m), while range from the person's box size had a median error of 0.25 m, so we dropped it.
- **Nodding the rig for height distorted the 3D view.** Tilting the handheld LiDAR up and down while the map was paused was meant to add height to the 3D view, but it distorted the point cloud too much, so we dropped it. Only a guard that discards scans tilted past 30° remains.
- **Power.** The Pi rebooted several times on a weak power bank. The spotter now runs only on the official Pi PSU or a 27 W USB-C PD bank, with detection at 5 fps, the camera at 10 fps and Bluetooth off (CPU 64% to 40%, 3.5 W to 3.1 W, no under-voltage).
- **A relay that reset after a button press.** B2 dropped out twice in the field. The cause: RadioHead's LoRa interrupt handler used the SPI bus that the screen shares, which tripped the ESP32's interrupt watchdog. We now poll the LoRa radio from the main loop; 11 presses later, 0 drops. Relays also restart themselves in about 3 s if their radio or main loop stalls.
- **Bandwidth.** Pictures, LiDAR and the map had to share one relay chain, so we prioritized the map and the people and let the images degrade.

## Accomplishments we're proud of

- End to end in under a day: from a LoRa alert on the spotter to the command post through three relays in under a second.
- A survivor's iPhone can send an SOS and get a reply with no app and no internet.
- Pose detection on thermal images, so detection works without light.
- The mesh routes around a dead relay automatically.

## What we learned

Under degraded comms, sending less matters more than sending faster. Splitting critical alerts from nice-to-have data, and degrading the second one gracefully, is what keeps the system useful when links get bad.

## Phone localization: where it stands

**Today.** Only after an SOS (no listening otherwise), B1 reports the phone's MAC tail and its own signal, and B2 and BASE listen 5 s for that phone's Wi-Fi frames. The dashboard draws a search area: a 1/d²-weighted centre of the relays that heard the phone, so it always lies among them, with a radius that shrinks as the same phone sends more SOS (by √k, to 3 m at least). Verified on Nov 8 with an iPhone: B2 heard 143 frames and BASE 162, and all three readings reached the SOS card. On that data the centre sits 0.6 m from B2; an earlier least-squares fit had put it 8 m beyond B2. It has not been tested yet with relays marked at different spots. Other phones are only counted per relay over BLE and shown as a zone.

**What we ruled out.**
- Tracking phones over BLE: about 50 phones were nearby at the venue, iPhone BLE addresses rotate about every 15 minutes, and a phone's BLE address cannot be linked to its Wi-Fi one.
- Listening from the walking spotter: the Pi 5's onboard Wi-Fi has no monitor mode.

**Next.** An ESP32 on the spotter that listens for the SOS phone while walking, so one phone is measured from many known positions; this also breaks the mirror ambiguity of a straight relay chain.

## What's next

- LoRa on every relay, and a spotter that falls back to any relay it can reach. Today B2 is the only gateway: it alone has the LoRa radio and the bulk-link access point, so losing B2 cuts the spotter off.
- UWB ranging between relays for better positions.
- SLAM running on the spotter, so it keeps a map even when the link drops.
- Gas and sound (knocking) sensors on the relays.
- Rugged, cheap enclosures.
- Multiple spotters sharing one map.
- Calibrated radio models for relay planning, from measured links.
