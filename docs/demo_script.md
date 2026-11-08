# Live demo script (3-4 minutes)

Judging is a live demo against four criteria: real-world relevance, a working live demonstration, deployability and ideation. Show the thing working. Keep the talk short.

## Setup (before the judges arrive)

- Relays placed:
  - **B2** about 5-8 m into the "building" (the hallway);
  - **B1** between B2 and the table;
  - **BASE** on the laptop at the table.
- All three are on LiPo or USB power, with screens on.
- Laptop: board hub running, http://localhost:5050 full screen. Clear the floor plan with **Reset map**, and clear old alerts.
- Spotter: official Pi PSU or a 27 W USB-C PD power bank only (weaker banks rebooted the Pi), charged. Check `rescue-spotter` with `systemctl status`. The debug view is not needed.
- A teammate or volunteer plays the casualty and lies on the floor out of sight, around a corner from the table. Dim the lights there if possible.
- Press **D2 on BASE** at the start (spotter held next to BASE), so the command post is the first node on the map.

## Demo rules (from the spotter tests)

- "Lying spot: about 1.7 m in front of the rig, inside the camera view, with clear floor behind (not against the wall). At 2.4 m against the wall the body runs along the bottom edge of the image and the detector loses it for several seconds."
- "Keep clear of the rig's left edge: nobody crouches or leans in toward the rig at its left side during the run; a body bent toward the camera there can read as a person down."
- Press D2 on each relay (B1, B2) as it is placed, with the spotter held next to it.
- To set the rig down (for example to free your hands), press **Pause map** first and **Resume map** after picking it up. Mapping continues from the same pose.
- One iPhone ready for the SOS step. Its Wi-Fi must not be auto-joined to the venue network.
- Fallbacks are ready in a second browser tab:
  - `python -m dashboard.app --replay logs/messages-<date>.jsonl` (recorded alert stream);
  - `python -m mapping.replay <walk>.jsonl.gz out.png` (recorded walk floor plan).

## Spotter beep patterns

The Pi's GPIO speaker beeps instead of speaking:

| Prompt | Pattern |
|---|---|
| Drop B1 here, then press D2 | 1 long |
| Drop B2 here, then press D2 | 2 long |
| B1 placed (D2 received) | 1 medium |
| B2 placed (D2 received) | 2 medium |
| B1, still in hand, out of range of BASE: step back | 3 short |
| B2, still in hand, out of range of both B1 and BASE: step back | 4 short |

## Timeline

| Time | Operator does | Judges see on the dashboard | Say |
|---|---|---|---|
| 0:00-0:20 | Stand at the table with the spotter. | The relay chain is green: SP1 → B2 → B1 → BASE, with heartbeats, RSSI and each relay's battery level. | "The building has no GPS, no cell and no Wi-Fi. This spotter and these three relays are the whole network. Small alerts go over LoRa and a mesh; a faster link carries pictures and the map." |
| 0:20-1:10 | Walk slowly down the hallway past B2, holding the rig level. | The floor plan grows wall by wall, with the walked path as a line. The live LiDAR, camera and thermal panes update. | "This is a 2D LiDAR SLAM running as I walk, so the command post gets a floor plan of a building nobody has seen. The accelerometer throws away scans when I tilt the rig, so the map stays clean." |
| 1:10-1:30 | Place B1 and then B2 along the way. Hold the spotter next to each and press its **D2** button. | Each node jumps onto the map at the spotter's position. When the link weakens, the relay in hand shows DROP HERE, the spotter beeps (see the beep table) and the dashboard shows a PLACE banner (bench-verified only; skip if it does not appear). | "Every relay I drop gets registered on the map with one button press, so rescuers know where the network actually is." |
| 1:30-2:10 | Turn the corner and stop with the casualty about 1.7 m in front of the rig, lying on clear floor. | Within about 2 s (measured 1.73 s from the fall to the card), a PERSON alert at the top of the list, marked *down*, with camera and/or thermal. A person marker on the floor plan at the right spot. Thermal pane shows the skeleton. | "The Hailo accelerator runs pose detection on the camera and on the thermal camera, so it still works in the dark. Someone lying on the floor is ranked first. The camera gives the direction and the LiDAR the distance, so we can put the person on the map instead of just saying 'someone is near B2'." |
| 2:10-2:40 | (Volunteer) joins `RESCUE-TEAM-NEARBY` on the iPhone and sends an SOS ("1 person, injured, leg trapped"). Operator clicks **Reply: help is on the way** on the SOS card. | The SOS card arrives in about 1 s. After the reply, the phone shows "Received by rescuers". B2 and BASE listen 5 s for that phone and their readings join the SOS card (verified). A search area appears among the relays that heard it (a few metres, not a fix; not yet tested with relays at different spots). Another SOS from the same phone shrinks it. | "Survivors don't need an app. Any phone sees the rescue Wi-Fi, the page pops up, and they hear back that help knows." |
| 2:40-3:15 | **Hold D1 on B1 for 1 s** (relay off). Wait about 8 s. Hold D1 again. (D1 is ignored on B2. The dashboard relay off/on button does the same.) | B1's screen shows a large red RELAY OFF and it goes grey on the chain. Alerts keep arriving, and their route now skips B1: B2 → BASE directly. Bulk panes may pause while B1 is off. B1 re-joins about 2 s after it is switched back on. | "Relays fail. Each relay normally listens only to its neighbour. Once that neighbour has been silent for a few seconds, it accepts from anyone, so the flood routes around the dead one by itself. The spotter's radio also reboots itself if its USB link freezes." |
| 3:15-3:45 | Point at the suggested relay markers on the map. Optional: open the 3D view. | Spots labelled "place relay N" ahead of the last relay, where the link margin runs out. The 3D view hides floor and ceiling returns by default. | "From the map and the walls, it suggests where to drop the next breadcrumb before you lose the link. Everything here is off-the-shelf: a Pi, Feathers, LoRa. It's cheap enough to leave behind." |

## If something breaks

| Problem | What to do |
|---|---|
| Map does not grow | Say "the SLAM is replayed from a walk we recorded an hour ago" and show the replay PNG. Keep going with alerts and SOS. |
| No PERSON from the camera | Press **D0** on B2 to send a test PERSON. That shows the chain, and the detection pane still shows skeletons. |
| Bulk panes frozen / LINK DOWN | The bulk link is independent of the alerts; say so. The alert chain still works. If B2 rebooted, the Pi's Wi-Fi watchdog re-joins RESCUE-LINK (about 12 s) by itself. |
| Bulk link struggling (weak signal) | Use the LoRa-only button: alerts keep coming over LoRa, pictures and the map pause. Restore it and pictures return within about 12 s. |
| Dashboard stuck | Restart it through the board hub. Fallback: the `--replay` tab. |

## Anticipated judge questions

- **Range?** LoRa is set to 5 dBm on purpose to make a one-room demo look like a building. At full power (20 dBm) LoRa goes hundreds of metres indoors. The bulk link is ESP-NOW and is limited to roughly Wi-Fi range per hop.
- **How accurate is the person position?** It is bearing plus LiDAR range from the spotter's SLAM pose, so roughly a metre at room scale. The camera yaw calibration is a recorded fit, not a lab measurement.
- **Deployability?** Every part is off-the-shelf and battery-powered. Relays cost tens of dollars each. Nothing depends on the cloud. Next steps: ruggedized enclosures, UWB ranging between relays, and running the SLAM on the Pi so the spotter keeps its own map.
- **Why two links?** Alerts must arrive even when the link is terrible, so they are tiny and flooded. Pictures and maps are nice to have, so they ride a faster link that degrades gracefully.
