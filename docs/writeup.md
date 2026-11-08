# Breadcrumb Rescue Network: project story

## Inspiration

When a building collapses or a wildfire takes out the towers, rescuers lose the tools they rely on: GPS, cell service and Wi-Fi. The challenge asked how to locate, identify and rescue people in exactly those conditions. Cave and mine rescue teams already do the simplest version of this: they leave a trail of markers behind them. We wanted a trail that talks: cheap radio relays dropped along the way, carrying what a handheld sensor sees back to the command post.

## What it does

A responder carries a handheld **spotter**: a Raspberry Pi 5 with an AI accelerator, an RGB camera, a thermal camera, a 2D LiDAR and an accelerometer. As they walk, they drop **breadcrumb relays**.

- **Finds people, in the dark too.** Pose detection runs on the Hailo-8 on both the camera and the thermal image, and someone lying on the floor is flagged first. The camera gives the bearing and the LiDAR the range, so each person is placed on the map.
- **Maps the building.** Handheld 2D SLAM from the LiDAR builds a floor plan at the command post, with the responder's path. Each relay is registered on that map with one button press.
- **Gets the information out without infrastructure.** Alerts are tiny JSON messages sent over LoRa and an ESP-NOW flood mesh. When a relay dies, the mesh routes around it within seconds. A second, faster link (about 420 kbps over ESP-NOW) carries the live camera, thermal, LiDAR and map data. When the signal weakens, it drops the pictures first, so the map and the people keep flowing.
- **Lets survivors call in.** Any phone near a relay sees the `RESCUE-TEAM-NEARBY` Wi-Fi, and a page pops up for sending an SOS (how many people, injuries, hazards). When the command post replies, the phone shows "Received by rescuers". Relays also count nearby phones over BLE.
- **Plans the next relay.** From the map's walls and the path walked so far, the dashboard suggests where to drop the next breadcrumb before the link runs out.

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
- **Hardware.** The AI HAT's stacking header had been mounted upside down, which cut off every GPIO pin until we found it. A speaker we added was too noisy to use, so we shelved it.
- **A handheld, tilted LiDAR is a hard SLAM input.** It sat 6° off level (we re-mounted it to about 3°), and people walk unevenly. We gate scans on the accelerometer and reject bad matches rather than corrupt the map.
- **Monocular depth did not beat simple geometry.** We tried Depth Anything V2 Small, scaled by the LiDAR, to range people. On 25 live observations it read a median 50% too far (median error 1.0 m), while range from the person's box size had a median error of 0.25 m, so we dropped it.
- **Nodding the rig for height distorted the 3D view.** Tilting the handheld LiDAR up and down while the map was paused was meant to add height to the 3D view, but it distorted the point cloud too much, so we dropped it. Only a guard that discards scans tilted past 30° remains.
- **Bandwidth.** Pictures, LiDAR and the map had to share one relay chain, so we prioritized the map and the people and let the images degrade.

## Accomplishments we're proud of

- End to end in under a day: from a LoRa alert on the spotter to the command post through three relays in under a second.
- A survivor's iPhone can send an SOS and get a reply with no app and no internet.
- Pose detection on thermal images, so detection works without light.
- The mesh routes around a dead relay automatically.

## What we learned

Under degraded comms, sending less matters more than sending faster. Splitting critical alerts from nice-to-have data, and degrading the second one gracefully, is what keeps the system useful when links get bad.

## Next step: phone localization by trilateration

**Today.** B2 scans BLE and reports aggregates about every 10 s: devices heard, strongest RSSI, number of Apple devices, and devices above -45 dBm. The SOS portal runs on B1. On the floor plan, phones are shown as a zone around the relay that heard them, not as a position; that view is being added now.

**Design.**
- Every breadcrumb (BASE, B1, B2) reports its strongest K devices, each with its own RSSI and a short hash of its BLE address.
- The command post matches the same address across nodes within a few seconds. It solves a weighted least-squares fix from the node positions recorded with the D2 button.
- The spotter's Pi also scans BLE while walking, so one device is measured from many known poses. This breaks the mirror ambiguity of a straight relay chain.
- Expected indoor error is about 3 to 5 m.

**Known limits.**
- BLE scanning shares the radio with ESP-NOW and Wi-Fi, so it lowers bulk throughput on a forwarding relay.
- iPhone BLE addresses rotate about every 15 minutes.
- An SOS phone's Wi-Fi and BLE addresses cannot be linked to each other.
- SP1 (RP2040 + LoRa) has no BLE.

## What's next

- UWB ranging between relays for better positions.
- SLAM running on the spotter, so it keeps a map even when the link drops.
- Gas and sound (knocking) sensors on the relays.
- Rugged, cheap enclosures.
- Multiple spotters sharing one map.
- Calibrated radio models for relay planning, from measured links.
