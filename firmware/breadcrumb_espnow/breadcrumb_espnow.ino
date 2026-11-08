// Breadcrumb Rescue Network: ESP-NOW relay node
// Board: Adafruit Feather ESP32-S3 Reverse TFT (#5691)
//
// One sketch for every ESP32 node. Set NODE_ID below (or pass -DNODE_ID=\"B2\" at build time).
//   "B2" : gateway relay (LoRa receiver + BLE phone listener)
//   "B1" : relay + rescue Wi-Fi captive portal "RESCUE-TEAM-NEARBY" (POST /sos -> SOS message)
//   (flooding with de-dup, hop limit, random delay, heartbeat)
//   Demo chain with 3 ESP32s: SP1 --LoRa--> B2 --ESP-NOW--> B1 --ESP-NOW--> CP
//   (old names: B3 -> B2, B2 -> B1)
//   "BASE"           : base station at the dashboard laptop; prints every new message to USB serial as one
//                      JSON line (plus its own HB every 15 s). Debug lines on BASE start with '#'.
// Contract: Shivansh's FIRMWARE_SPEC.md (messages < 200 bytes, unknown fields forwarded unchanged).
//
// Message format (Shivansh's plan, short-key JSON, < 250 bytes for ESP-NOW):
//   {"id":"a3f1","src":"SP1","ttl":4,"rt":["B2","B1"],"t":"PERSON","c":88,"th":1,"z":"B2"}
// Each relay appends its own ID to "rt" and adds "q": RSSI (dBm) of the hop it heard.
//
// Buttons: D0 (active low) sends a TEST PERSON alert from this node.
//          D1 held 1 s switches this relay off / on (no radio TX at all: the chain heals around it). BASE, B2: ignored.
//          D2 sends MARK {"t":"MARK","z":<this node>}: the operator holds the spotter next to this node and
//          presses it; the dashboard registers the node at the spotter's current SLAM pose.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#ifndef NODE_ID
#define NODE_ID "B2"
#endif

// B2 is the gateway: LoRa receiver (RFM95W breakout) + BLE phone listener, on by default for B2.
// Wiring RFM95W -> Feather: VIN-3V, GND-GND, SCK-SCK, MISO-MI, MOSI-MO, CS-D10, RST-D11, G0-D9
#ifndef HAS_LORA
#define HAS_LORA (__builtin_strcmp(NODE_ID, "B2") == 0)
#endif
#ifndef HAS_BLE
// BLE phone count off by default (build with -DHAS_BLE=1 to get it back on B2): B2 is the only gateway, and with
// ~55 phones advertising it self-restarted 4x in 5 min (11:25-11:29 EST 2026-11-08, radio sends stalled 15 s).
#define HAS_BLE 0
#endif
#include <SPI.h>
#include <RH_RF95.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <DNSServer.h>
#include <WebServer.h>
#include "portal_pages.h"

// B1 runs the rescue Wi-Fi captive portal (soft-AP on the ESP-NOW channel): a phone joins
// RESCUE-TEAM-NEARBY, the form pops up, POST /sos sends {"t":"SOS","msg":...} into the mesh.
#ifndef HAS_PORTAL
#define HAS_PORTAL (__builtin_strcmp(NODE_ID, "B1") == 0)
#endif
static const char* PORTAL_SSID = "RESCUE-TEAM-NEARBY";
static const int SOS_MSG_MAX = 100;           // spec: msg at most 100 characters
static const uint32_t SOS_MIN_GAP_MS = 3000;  // ignore double taps / resubmits
DNSServer dns;
WebServer web(80);
static uint32_t lastSos = 0;
struct SosRec { char id[5]; bool acked; char reply[48]; uint32_t at, lastSl; };
// While the phone keeps the "sent" page open it polls /status every 2 s; until the ACK, every SL_REPEAT_MS
// B1 re-reads the phone RSSI and sends a new SL, so the dashboard search area keeps updating (stops after SL_REPEAT_FOR_MS).
static const uint32_t SL_REPEAT_MS = 20000, SL_REPEAT_FOR_MS = 600000;
static SosRec sosLog[8];
static int sosNext = 0;
static const int LORA_CS = 10, LORA_RST = 11, LORA_INT = 9;
static const float LORA_FREQ = 915.0;
// RadioHead normally reads the radio over SPI inside the DIO0 interrupt. The TFT shares that SPI bus, so a packet
// arriving while loop() draws the screen left the ISR waiting for the bus lock with interrupts off: interrupt
// watchdog reset (reset reason 5, B2 on the bench at 10:24 EST, right after it forwarded an SP1 packet).
// B2 only receives, so no interrupt: loop() polls DIO0 (RxDone, high until the IRQ flags are cleared).
class RF95Polled : public RH_RF95 {
 public:
  explicit RF95Polled(uint8_t cs) : RH_RF95(cs, RH_INVALID_PIN) {}
  void poll() { handleInterrupt(); }
};
RF95Polled rf95(LORA_CS);
static bool loraOk = false;

// ---- demo topology: which neighbour each node accepts from (chain SP1 -> B2 -> B1 -> CP) ----
// Upstream = farther from the command post. Downstream messages (ACK) are not used yet.
struct Link { const char* node; const char* acceptFrom[3]; };
static const Link TOPOLOGY[] = {
  {"B2", {"SP1", "GW", nullptr}},
  {"B1", {"B2", nullptr, nullptr}},
  {"BASE", {"B1", nullptr, nullptr}},
};

// Downlink (ACK from BASE back toward the survivor): B1 accepts from BASE, B2 from B1.
static const Link DOWNLINK[] = {
  {"B1", {"BASE", nullptr, nullptr}},
  {"B2", {"B1", nullptr, nullptr}},
};

static const uint8_t WIFI_CHANNEL = 1;
static const int DEFAULT_TTL = 4;
static const uint32_t HEARTBEAT_MS = 15000;
static const uint32_t DELAY_MIN_MS = 50, DELAY_MAX_MS = 300;
static const int SEEN_SIZE = 50;
static const int QUEUE_SIZE = 16;
static const int MAX_LEN = 250;      // ESP-NOW frame limit (receive buffer)
static const int MSG_MAX = 199;      // contract: every message < 200 bytes

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// ---- received-packet queue (filled in the Wi-Fi callback, drained in loop) ----
struct Packet { char data[MAX_LEN + 1]; int len; int rssi; };
static Packet rxQueue[QUEUE_SIZE];
static volatile int rxHead = 0, rxTail = 0;
static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// ---- pending re-broadcasts (random delay) ----
struct Pending { char data[MAX_LEN + 1]; uint32_t due; bool used; };
static Pending pending[QUEUE_SIZE];

static char seen[SEEN_SIZE][9];
static int seenNext = 0;

static bool isCP = false;
static bool demoTopology = true;
static uint32_t lastHeartbeat = 0;
static uint32_t cntRx = 0, cntFwd = 0, cntDup = 0, cntDrop = 0, cntHeal = 0;
static char lastType[10] = "-", lastFrom[8] = "-";
static int lastRssi = 0;
static bool screenDirty = true;

// ---- demo switches ----
static bool relayOff = false;    // D1 (or CMD off): this relay plays dead, no radio TX at all -> self-heal demo
static bool loraOnly = false;    // B2, CMD lora: drop the Pi's bulk link (weak signal); alerts stay on LoRa

// ---- battery (MAX17048 fuel gauge on the Feather, I2C 0x36) ----
#include <Wire.h>
static bool gaugeOk = false;
static int batPct = -1;
static float batV = 0;

// ---- relay placement prompt: the relay in the operator's hand watches the beacon RSSI of the last placed
// node (B1 watches BASE, B2 watches B1). Weak for a few samples -> PLACE ("drop me here, press D2"),
// repeated until D2. Neither too close (wasted range) nor too far (12 Mbps bulk needs about -85 dBm).
static int placeTh = -70;                       // dBm, EMA of the upstream beacon; CMD th changes it live
static const int PLACE_HYST = 4, PLACE_N = 3;   // stop prompting above th + 4; prompt after 3 weak samples
static const uint32_t PLACE_REPEAT_MS = 15000;
static bool placed = false;                     // D2 pressed on this relay
static float upEma = 0;
static int weakN = 0;
static uint32_t lastPlaceMsg = 0, upLastSeen = 0;
static bool prompting = false, upLost = false;
static int upCode = 0;                          // the node behind this relay that placeLoop watches

// ---- relay self-restart: B2 is the spotter's only gateway (LoRa + RESCUE-LINK), so a relay whose radio stops
// completing ESP-NOW sends (beacon every 2 s) or whose loop stops for 10 s restarts itself (about 3 s) instead of
// staying dark until someone presses RESET. Checked from a FreeRTOS timer, so a stuck loop() cannot hide it.
// D2 "placed", LoRa-only and th survive our own restart (RTC memory, cleared by a power cut).
static volatile uint32_t lastTxDone = 0, lastLoopAt = 0;
static const uint32_t TX_STUCK_MS = 30000, LOOP_STUCK_MS = 10000;   // 15 s fired 4x on B2 11:25-11:29 EST (walk, ~55 BLE phones)
static const uint32_t KEEP_MAGIC = 0xB2EE5AFE;
RTC_NOINIT_ATTR static uint32_t keepMagic;
RTC_NOINIT_ATTR static int32_t keepPlaced, keepLora, keepTh, keepWhy;
static int wdWhy = 0;                           // after our restart: 1 radio stuck, 2 loop stuck (first HB "wd")
static volatile uint8_t loopStage = 0;         // which part of loop() runs now (first HB "ws" after a restart)
RTC_NOINIT_ATTR static int32_t keepGap, keepStage;
static int wdGap = 0, wdStage = 0;
static void wdRestart(int why) {
  keepPlaced = placed; keepLora = loraOnly; keepTh = placeTh; keepWhy = why; keepStage = loopStage;
  keepGap = (int32_t)(millis() - (why == 1 ? lastTxDone : lastLoopAt));
  keepMagic = KEEP_MAGIC;
  esp_restart();
}
static void wdTick(TimerHandle_t) {
  uint32_t now = millis();
  if (relayOff) { lastTxDone = lastLoopAt = now; return; }   // D1: silent on purpose
  // signed: loop() / the send callback run on other tasks and may stamp a millisecond later than `now`
  // (unsigned now - later = 4e9 -> a false "stuck" restart, seen once on the bench at 10:12 EST)
  if ((int32_t)(now - lastLoopAt) > (int32_t)LOOP_STUCK_MS) wdRestart(2);
  else if ((int32_t)(now - lastTxDone) > (int32_t)TX_STUCK_MS) wdRestart(1);
}
static const char* const NODE_NAME[] = {"BASE", "B1", "B2", "Pi"};   // by code (bulk-link codes)

// ---- SOS phone localization: after an SOS, B1 sends SL with the phone's MAC tail; every other node listens
// for that phone's Wi-Fi frames (promiscuous, data frames, transmitter address) for SNIFF_MS and answers SLR.
// Only after an SOS: no listening otherwise.
static const uint32_t SNIFF_MS = 5000;
static volatile uint8_t sniffMac[3];
static volatile int32_t sniffSum = 0;
static volatile int sniffN = 0;
static uint32_t sniffUntil = 0;
static char sniffPm[7] = "";
static char localSeen[5] = "";                  // id of the last SL/PLACE acted on (both copies of a resend)

// ---- BLE phone listener: unique addresses above an RSSI threshold, strongest RSSI, Apple count ----
// Spec says RSSI > -90; a crowded venue saturates that, so -75 (~a few metres) by default.
static const uint32_t BLE_REPORT_MS = 10000;
static const int BLE_MAX = 128;
#ifndef BLE_MIN_RSSI
#define BLE_MIN_RSSI -75
#endif
static uint32_t bleSeen[BLE_MAX];
#ifndef NEAR_RSSI
#define NEAR_RSSI -45   // "right next to the breadcrumb" (~0.5 m): the hidden-phone demo
#endif
static const int NEAR_MAX = 16;
static uint32_t nearSeen[NEAR_MAX];
static int bleCount = 0, bleApple = 0, bleBest = -127, bleNear = 0;
static portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
static BLEScan* bleScan = nullptr;

class PhoneCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    int rssi = dev.getRSSI();
    if (rssi <= BLE_MIN_RSSI) return;
    bool apple = false;
    if (dev.haveManufacturerData()) {
      String m = dev.getManufacturerData();
      apple = m.length() >= 2 && (uint8_t)m[0] == 0x4C && (uint8_t)m[1] == 0x00;  // company 0x004C
    }
    const uint8_t* a = (const uint8_t*)dev.getAddress().getNative();
    uint32_t h = ((uint32_t)a[2] << 24) | ((uint32_t)a[3] << 16) | ((uint32_t)a[4] << 8) | a[5];
    portENTER_CRITICAL(&bleMux);
    bool found = false;
    for (int i = 0; i < bleCount; i++) if (bleSeen[i] == h) { found = true; break; }
    if (!found && bleCount < BLE_MAX) { bleSeen[bleCount++] = h; if (apple) bleApple++; }
    if (rssi > NEAR_RSSI) {
      bool nf = false;
      for (int i = 0; i < bleNear; i++) if (nearSeen[i] == h) { nf = true; break; }
      if (!nf && bleNear < NEAR_MAX) nearSeen[bleNear++] = h;
    }
    if (rssi > bleBest) bleBest = rssi;
    portEXIT_CRITICAL(&bleMux);
  }
};

// ---------------------------------------------------------------------------
static void randomId(char* out) {
  static const char hex[] = "0123456789abcdef";
  uint32_t r = esp_random();
  for (int i = 0; i < 4; i++) out[i] = hex[(r >> (i * 4)) & 0xF];
  out[4] = 0;
}

static bool alreadySeen(const char* id) {
  for (int i = 0; i < SEEN_SIZE; i++)
    if (seen[i][0] && strncmp(seen[i], id, 8) == 0) return true;
  return false;
}

static void remember(const char* id) {
  strncpy(seen[seenNext], id, 8);
  seen[seenNext][8] = 0;
  seenNext = (seenNext + 1) % SEEN_SIZE;
}

static bool nodeAlive(const char* name);   // beacon heard in the last few seconds (bulk-link section)

// Demo topology: accept only from the configured neighbour -- unless every configured neighbour has gone
// silent (no beacon for 6 s). Then accept from anyone, so the flood routes around a dead relay by itself.
static bool acceptLinks(const Link& l, const char* hop) {
  bool anyAlive = false;
  for (const char* a : l.acceptFrom) {
    if (!a) continue;
    if (strcmp(a, hop) == 0) return true;
    if (nodeAlive(a)) anyAlive = true;
  }
  if (!anyAlive) cntHeal++;
  return !anyAlive;
}

static bool acceptFrom(const char* hop, bool downlink) {
  if (!demoTopology) return true;
  if (downlink) {
    for (const Link& l : DOWNLINK) {
      if (strcmp(l.node, NODE_ID) != 0) continue;
      return acceptLinks(l, hop);
    }
    return false;  // BASE (or unlisted node) never takes downlink traffic
  }
  for (const Link& l : TOPOLOGY) {
    if (strcmp(l.node, NODE_ID) != 0) continue;
    return acceptLinks(l, hop);
  }
  return true;  // node not listed: accept everything
}

static void sendRaw(const char* s, int len) {
  if (relayOff) return;                         // switched off: silent (receives, never transmits)
  esp_now_send(BROADCAST, (const uint8_t*)s, len);
}

// ---- bulk link (camera / thermal / LiDAR): Pi --Wi-Fi UDP--> B2 --ESP-NOW unicast--> B1 --> BASE ----
// Binary frames, first byte 0xB7 (mesh JSON always starts with '{'):
//   beacon: B7 00 <code>                                   every node, every 2 s (next-hop discovery)
//   chunk : B7 01 <hop> <kind> <seq lo hi> <idx lo hi> <cnt lo hi> payload   hop = code of the sender
// Codes: BASE 0, B1 1, B2 2, Pi 3. A chunk always moves to a lower code: the next lower node if it was
// heard in the last 6 s, otherwise the lowest node heard (B1 down -> B2 sends straight to BASE).
// Unicast, so the radio retries lost frames; alerts keep using the flooding mesh (and LoRa) above.
#include <AsyncUDP.h>
#include "mbedtls/base64.h"
#ifndef HAS_LINK_AP
#define HAS_LINK_AP (__builtin_strcmp(NODE_ID, "B2") == 0)
#endif
static const uint8_t BULK_MAGIC = 0xB7;
static const char* LINK_SSID = "RESCUE-LINK";      // hidden, WPA2: only the spotter Pi joins it
static const char* LINK_PASS = "breadcrumb-link";
static const uint16_t LINK_PORT = 5005;
static const uint32_t BEACON_MS = 2000, NB_ALIVE_MS = 8000;
static const int BULK_Q = 96, NB_MAX = 4;
struct Nb { uint8_t mac[6]; volatile uint32_t seen; volatile int rssi; bool peer; };
static Nb nb[NB_MAX];
struct Bulk { uint8_t d[MAX_LEN]; uint8_t len; };
static Bulk bulkIn[BULK_Q], bulkOut[BULK_Q];
static volatile int binHead = 0, binTail = 0;
static int boutHead = 0, boutTail = 0;
static portMUX_TYPE bulkMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool bulkInflight = false;
static uint32_t bulkSentAt = 0, lastBeacon = 0;
static uint32_t bulkRx = 0, bulkFwd = 0, bulkDrop = 0, bulkFail = 0, bulkUdp = 0, bulkNoRoute = 0, bulkBusy = 0;
static int bulkVia = -1;
static AsyncUDP linkUdp;   // packets handled in the network task: a slow loop() (screen redraw) must not
                           // let lwIP's tiny UDP receive mailbox overflow
static volatile uint32_t piIp4 = 0;        // the spotter Pi on RESCUE-LINK (source address of its bulk packets)
static const uint16_t PI_SAY_PORT = 5006;  // the Pi's prompt listener: SAY (speaker) and MODE
static void toPi(JsonDocument& d) {        // B2 only: one JSON datagram to the Pi
  if (!HAS_LINK_AP || relayOff) return;
  char buf[160];
  size_t n = serializeJson(d, buf, sizeof(buf));
  uint32_t ip = piIp4 ? (uint32_t)piIp4 : (uint32_t)IPAddress(192, 168, 4, 2);
  linkUdp.writeTo((const uint8_t*)buf, n, IPAddress(ip), PI_SAY_PORT);
  Serial.printf("# to pi %s\n", buf);
}
static void sayToPi(const char* k, const char* m) {
  JsonDocument d;
  d["t"] = "SAY";
  d["k"] = k;
  d["m"] = m;
  toPi(d);
}

static int myCode() {
  if (isCP) return 0;
  if (strcmp(NODE_ID, "B1") == 0) return 1;
  if (strcmp(NODE_ID, "B2") == 0) return 2;
  return 9;
}

// called from the Wi-Fi task (ESP-NOW receive callback)
static void bulkOnRecv(const uint8_t* mac, const uint8_t* data, int len, int rssi) {
  if (len == 3 && data[1] == 0) {
    int c = data[2];
    if (c < NB_MAX && mac) { memcpy(nb[c].mac, mac, 6); nb[c].seen = millis(); nb[c].rssi = rssi; }
    return;
  }
  if (len < 10 || data[1] != 1) return;
  portENTER_CRITICAL_ISR(&bulkMux);
  int next = (binHead + 1) % BULK_Q;
  if (next != binTail) {
    memcpy(bulkIn[binHead].d, data, len);
    bulkIn[binHead].len = len;
    binHead = next;
  } else bulkDrop++;
  portEXIT_CRITICAL_ISR(&bulkMux);
}

static void bulkOnSent(const esp_now_send_info_t* info, esp_now_send_status_t st) {
  if (st != ESP_NOW_SEND_SUCCESS) bulkFail++;
  bulkInflight = false;
  lastTxDone = millis();                                     // the radio finished a send (self-restart check)
}

static int codeOf(const char* name) {
  if (strcmp(name, "BASE") == 0) return 0;
  if (strcmp(name, "B1") == 0) return 1;
  if (strcmp(name, "B2") == 0) return 2;
  return -1;                                                 // SP1 (LoRa only) etc.: not tracked
}

static bool nodeAlive(const char* name) {
  int c = codeOf(name);
  if (c < 0 || c >= NB_MAX) return true;
  return nb[c].seen && millis() - nb[c].seen < NB_ALIVE_MS;
}

static int nextHop() {
  int mine = myCode();
  uint32_t now = millis();
  auto alive = [&](int c) { return nb[c].seen && now - nb[c].seen < NB_ALIVE_MS; };
  if (mine >= 1 && alive(mine - 1)) return mine - 1;       // the chain: next node toward BASE
  for (int c = 0; c < mine && c < NB_MAX; c++) if (alive(c)) return c;   // skip a dead node
  return bulkVia;                                            // beacons lost in a busy moment: keep the last hop
}

static void bulkQueueOut(const uint8_t* d, int len) {
  int next = (boutHead + 1) % BULK_Q;
  if (next == boutTail) { bulkDrop++; return; }
  memcpy(bulkOut[boutHead].d, d, len);
  bulkOut[boutHead].len = len;
  boutHead = next;
}

static void bulkHandle(uint8_t* d, int len) {
  int hop = d[2], mine = myCode();
  if (mine >= hop) return;                                   // only ever toward BASE
  bulkRx++;
  if (isCP) {
    static uint32_t recent[32];                              // radio retries can deliver a frame twice
    static int rn = 0;
    uint32_t key = ((uint32_t)d[3] << 24) ^ ((uint32_t)(d[4] | d[5] << 8) << 10) ^ (uint32_t)(d[6] | d[7] << 8);
    for (uint32_t k : recent) if (k == key) return;
    recent[rn] = key; rn = (rn + 1) % 32;
    static unsigned char b64[340];
    size_t olen = 0;
    if (mbedtls_base64_encode(b64, sizeof(b64), &olen, d + 10, len - 10) != 0) return;
    b64[olen] = 0;
    Serial.printf("$B %u %u %u %u %s\n", d[3], d[4] | d[5] << 8, d[6] | d[7] << 8, d[8] | d[9] << 8, b64);
    return;
  }
  d[2] = mine;
  bulkQueueOut(d, len);
}

static void bulkLoop() {
  uint32_t now = millis();
  if (relayOff) {                                            // dead relay: no beacon, no forwarding
    portENTER_CRITICAL(&bulkMux);
    binTail = binHead;
    portEXIT_CRITICAL(&bulkMux);
    boutTail = boutHead;
    return;
  }
  if (now - lastBeacon >= BEACON_MS) {                       // beacon: "I am code N" (BASE included)
    lastBeacon = now;
    uint8_t b[3] = {BULK_MAGIC, 0, (uint8_t)myCode()};
    esp_now_send(BROADCAST, b, 3);
  }
  for (int c = 0; c < NB_MAX; c++) {                         // unicast needs a registered peer
    if (nb[c].seen && !nb[c].peer) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, nb[c].mac, 6);
      peer.channel = 0;
      peer.ifidx = WIFI_IF_STA;
      peer.encrypt = false;
      nb[c].peer = esp_now_add_peer(&peer) == ESP_OK || esp_now_is_peer_exist(nb[c].mac);
      if (nb[c].peer) {                                      // bulk unicast at 12 Mbps OFDM instead of 1 Mbps;
        esp_now_rate_config_t rc = {};                       // broadcasts (alerts, beacons) stay at 1 Mbps
        rc.phymode = WIFI_PHY_MODE_11G;
        rc.rate = WIFI_PHY_RATE_12M;
        esp_err_t e = esp_now_set_peer_rate_config(nb[c].mac, &rc);
        Serial.printf("# peer %d rate 12M %s\n", c, e == ESP_OK ? "ok" : esp_err_to_name(e));
      }
    }
  }
  while (true) {                                             // chunks from the mesh
    Bulk b;
    bool have = false;
    portENTER_CRITICAL(&bulkMux);
    if (binTail != binHead) { b = bulkIn[binTail]; binTail = (binTail + 1) % BULK_Q; have = true; }
    portEXIT_CRITICAL(&bulkMux);
    if (!have) break;
    bulkHandle(b.d, b.len);
  }
  if (boutTail != boutHead && (!bulkInflight || now - bulkSentAt > 60)) {
    int nh = nextHop();
    if (nh < 0 || !nb[nh].peer) {                            // nobody to send to: drop stale chunks
      boutTail = (boutTail + 1) % BULK_Q;
      bulkNoRoute++;
      return;
    }
    bulkVia = nh;
    Bulk& b = bulkOut[boutTail];
    if (esp_now_send(nb[nh].mac, b.d, b.len) == ESP_OK) {
      bulkInflight = true;
      bulkSentAt = now;
      boutTail = (boutTail + 1) % BULK_Q;
      bulkFwd++;
    } else bulkBusy++;
  }
  static uint32_t lastStat = 0;                              // '#' line: ignored by the dashboard
  if (now - lastStat >= 5000 && (bulkRx || bulkFwd || cntHeal)) {
    lastStat = now;
    int q = (boutHead - boutTail + BULK_Q) % BULK_Q;
    Serial.printf("# bulk udp %lu rx %lu fwd %lu fail %lu drop %lu noroute %lu busy %lu q %d via %d heal %lu\n",
                  (unsigned long)bulkUdp, (unsigned long)bulkRx, (unsigned long)bulkFwd, (unsigned long)bulkFail,
                  (unsigned long)bulkDrop, (unsigned long)bulkNoRoute, (unsigned long)bulkBusy, q, bulkVia,
                  (unsigned long)cntHeal);
  }
}

// ---------------------------------------------------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  int rssi = 0;
#endif
  if (len <= 0 || len > MAX_LEN) return;
  if (data[0] == BULK_MAGIC) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    bulkOnRecv(info->src_addr, data, len, rssi);
#else
    bulkOnRecv(mac, data, len, rssi);
#endif
    return;
  }
  portENTER_CRITICAL_ISR(&rxMux);
  int next = (rxHead + 1) % QUEUE_SIZE;
  if (next != rxTail) {
    memcpy(rxQueue[rxHead].data, data, len);
    rxQueue[rxHead].data[len] = 0;
    rxQueue[rxHead].len = len;
    rxQueue[rxHead].rssi = rssi;
    rxHead = next;
  }
  portEXIT_CRITICAL_ISR(&rxMux);
}

static void schedule(const char* s) {
  for (Pending& p : pending) {
    if (!p.used) {
      strncpy(p.data, s, MAX_LEN);
      p.data[MAX_LEN] = 0;
      p.due = millis() + random(DELAY_MIN_MS, DELAY_MAX_MS + 1);
      p.used = true;
      return;
    }
  }
  cntDrop++;  // queue full
}

// Originate a message from this node (heartbeat, test alert)
static void originate(JsonDocument& doc, bool resend = false, bool radio = false) {
  char id[5];
  randomId(id);
  doc["id"] = id;
  doc["src"] = NODE_ID;
  doc["ttl"] = DEFAULT_TTL;
  doc["rt"].to<JsonArray>();
  char buf[MAX_LEN + 1];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0 || n > MSG_MAX) return;
  remember(id);
  if (isCP && !radio) Serial.println(buf);
  else {
    sendRaw(buf, n);
    if (resend) schedule(buf);  // same id again 50-300 ms later: a lost frame gets a second chance
  }
}

// ---- captive portal (B1) ----
static SosRec* findSos(const char* id) {
  for (SosRec& r : sosLog) if (r.id[0] && strcmp(r.id, id) == 0) return &r;
  return nullptr;
}
static void portalRedirect() {
  web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  web.send(302, "text/plain", "");
}

// keep printable characters, cut at maxBytes on a UTF-8 boundary
static String cleanText(const String& in, size_t maxBytes) {
  String out;
  for (size_t i = 0; i < in.length(); i++) {
    uint8_t c = (uint8_t)in[i];
    if (c < 0x20) c = ' ';
    else if (c == '"') c = '\'';   // no JSON escaping: keeps the message under 200 bytes
    else if (c == '\\') c = '/';
    out += (char)c;
  }
  out.trim();
  if (out.length() > maxBytes) {
    size_t n = maxBytes;
    while (n > 0 && ((uint8_t)out[n] & 0xC0) == 0x80) n--;  // don't split a multi-byte char
    out = out.substring(0, n);
  }
  return out;
}

// RSSI (dBm) at this AP of the phone that is posting the SOS, from the station list; 0 = unknown.
// One phone connected: it is that one. Several: match the HTTP client's IP through the DHCP leases.
static int sosPhoneRssi(uint8_t* macOut) {
  wifi_sta_list_t sl;
  if (esp_wifi_ap_get_sta_list(&sl) != ESP_OK || sl.num <= 0) return 0;
  if (sl.num == 1) { memcpy(macOut, sl.sta[0].mac, 6); return sl.sta[0].rssi; }
  esp_netif_t* nif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
  if (!nif) return 0;
  int n = min((int)sl.num, (int)ESP_WIFI_MAX_CONN_NUM);
  esp_netif_pair_mac_ip_t pairs[ESP_WIFI_MAX_CONN_NUM];
  for (int i = 0; i < n; i++) memcpy(pairs[i].mac, sl.sta[i].mac, 6);
  if (esp_netif_dhcps_get_clients_by_mac(nif, n, pairs) != ESP_OK) return 0;
  uint32_t ip = (uint32_t)web.client().remoteIP();
  for (int i = 0; i < n; i++)
    if (pairs[i].ip.addr == ip) { memcpy(macOut, sl.sta[i].mac, 6); return sl.sta[i].rssi; }
  return 0;
}

static void sendSl(const uint8_t* pmac, int sr) {
  char pm[7];
  snprintf(pm, sizeof(pm), "%02x%02x%02x", pmac[3], pmac[4], pmac[5]);
  JsonDocument sl;
  sl["t"] = "SL";
  sl["z"] = NODE_ID;
  sl["pm"] = pm;
  sl["r"] = sr;
  originate(sl, true);
}

static void handleSos() {
  uint32_t now = millis();
  if (now - lastSos < SOS_MIN_GAP_MS && lastSos != 0) { portalRedirect(); return; }
  lastSos = now;
  // Tap answers (all optional) -> one short readable line for the dashboard, msg <= 100 bytes.
  int people = constrain(web.arg("p").toInt(), 0, 4);   // 0 = not answered, 4 = "4 or more"
  int inj = web.hasArg("inj") ? constrain(web.arg("inj").toInt(), 0, 2) : -1;
  String urgent, hazard;
  for (int i = 0; i < web.args(); i++) {
    String k = web.argName(i), v = cleanText(web.arg(i), 12);
    if (k == "u") urgent += (urgent.length() ? "," : "") + v;
    else if (k == "h") hazard += (hazard.length() ? "," : "") + v;
  }
  String where = cleanText(web.arg("msg"), 40);
  String msg;
  if (people) msg += people == 1 ? "1 person" : (people == 4 ? "4+ ppl" : String(people) + " ppl");
  if (inj == 0) msg += String(msg.length() ? ", " : "") + "no injuries";
  if (inj == 1) msg += String(msg.length() ? ", " : "") + "injured, can move";
  if (inj == 2) msg += String(msg.length() ? ", " : "") + "TRAPPED";
  // location before the lists: if everything is ticked, the 100-byte cut drops trailing hazards, not "where"
  if (where.length()) msg += String(msg.length() ? ". " : "") + "At: " + where;
  if (urgent.length()) msg += String(msg.length() ? ". " : "") + "URGENT: " + urgent;
  if (hazard.length()) msg += String(msg.length() ? ". " : "") + "Danger: " + hazard;
  if (!msg.length()) msg = "SOS, no details";
  msg = cleanText(msg, SOS_MSG_MAX);

  JsonDocument doc;
  doc["t"] = "SOS";
  doc["z"] = NODE_ID;
  doc["msg"] = msg;
  if (people) doc["p"] = people;
  if (inj >= 0) doc["inj"] = inj;  // 0 none, 1 hurt but mobile, 2 trapped
  uint8_t pmac[6] = {0};
  int sr = sosPhoneRssi(pmac);      // how close the phone is to this relay (distance ring on the floor plan)
  // only if it fits: originate adds id/src/ttl/rt (~40 B) and a relay hop adds rt + q (~15 B), limit MSG_MAX
  if (sr < 0 && measureJson(doc) + 70 <= (size_t)MSG_MAX) doc["sr"] = sr;
  originate(doc, true);
  if (sr < 0) sendSl(pmac, sr);     // ask BASE and B2 to listen for this phone: trilateration on the dashboard
  SosRec& rec = sosLog[sosNext];
  sosNext = (sosNext + 1) % 8;
  strncpy(rec.id, doc["id"] | "", 4); rec.id[4] = 0;
  rec.acked = false; rec.reply[0] = 0;
  rec.at = rec.lastSl = now;
  strncpy(lastType, "SOS", sizeof(lastType) - 1);
  strncpy(lastFrom, "web", sizeof(lastFrom) - 1);
  screenDirty = true;
  Serial.printf("# sos %s\n", msg.c_str());
  String page = FPSTR(PAGE_SENT);
  page.replace("%ID%", rec.id);
  web.send(200, "text/html", page);
}

// GET /status?id=xxxx -> {"ack":true,"reply":"..."} once the rescuers acknowledged
static void handleStatus() {
  SosRec* r = findSos(web.arg("id").c_str());
  uint32_t now = millis();
  if (r && !r->acked && now - r->at < SL_REPEAT_FOR_MS && now - r->lastSl >= SL_REPEAT_MS) {
    r->lastSl = now;
    uint8_t pmac[6] = {0};
    int sr = sosPhoneRssi(pmac);
    if (sr < 0) { sendSl(pmac, sr); Serial.printf("# sos %s: phone %d dBm, SL repeat\n", r->id, sr); }
  }
  JsonDocument d;
  d["ack"] = r && r->acked;
  if (r && r->acked) d["reply"] = r->reply;
  String out;
  serializeJson(d, out);
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json", out);
}

static void portalBegin() {
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());
  web.on("/", HTTP_GET, []() { web.send_P(200, "text/html", PAGE_FORM); });
  web.on("/sos", HTTP_POST, handleSos);
  web.on("/status", HTTP_GET, handleStatus);
  web.onNotFound(portalRedirect);  // /generate_204, /hotspot-detect.html, /connecttest.txt, ...
  web.begin();
  Serial.printf("# portal %s on %s\n", PORTAL_SSID, WiFi.softAPIP().toString().c_str());
}

// ---- SOS phone localization (promiscuous listen, only after an SOS) ----
static void sniffCb(void* buf, wifi_promiscuous_pkt_type_t type) {   // Wi-Fi task: keep it tiny
  if (type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t* p = (const wifi_promiscuous_pkt_t*)buf;
  if (p->rx_ctrl.sig_len < 24) return;
  const uint8_t* h = p->payload;                                      // 802.11 header: addr2 (transmitter) at 10
  if (h[13] != sniffMac[0] || h[14] != sniffMac[1] || h[15] != sniffMac[2]) return;
  sniffSum += p->rx_ctrl.rssi;
  sniffN++;
}

static void sniffStart(const char* pm) {
  if (strlen(pm) != 6 || sniffUntil) return;
  for (int i = 0; i < 3; i++) {
    char b[3] = {pm[2 * i], pm[2 * i + 1], 0};
    sniffMac[i] = (uint8_t)strtoul(b, nullptr, 16);
  }
  strncpy(sniffPm, pm, 6); sniffPm[6] = 0;
  sniffSum = 0; sniffN = 0;
  wifi_promiscuous_filter_t f = {.filter_mask = WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&f);
  esp_wifi_set_promiscuous_rx_cb(sniffCb);
  esp_wifi_set_promiscuous(true);
  sniffUntil = millis() + SNIFF_MS;
  if (!sniffUntil) sniffUntil = 1;
  Serial.printf("# sniff %s for %lu ms\n", sniffPm, (unsigned long)SNIFF_MS);
}

static void sniffLoop(uint32_t now) {
  if (!sniffUntil || (int32_t)(now - sniffUntil) < 0) return;
  esp_wifi_set_promiscuous(false);
  sniffUntil = 0;
  int n = sniffN;
  JsonDocument r;
  r["t"] = "SLR";
  r["z"] = NODE_ID;
  r["pm"] = sniffPm;
  r["n"] = n;
  if (n > 0) r["r"] = (int)lround((double)sniffSum / n);
  originate(r, true);
  Serial.printf("# sniff %s done: %d frames\n", sniffPm, n);
}

// B2 (next to the Pi): speak the placement prompt of a relay in the operator's hand
static void placeSay(const char* z, const char* up, bool lost) {
  char k[8], m[64];
  snprintf(k, sizeof(k), "%s%c", lost ? "lost" : "place", z[0] ? z[strlen(z) - 1] : '?');
  if (lost) snprintf(m, sizeof(m), "Relay %s lost %s. Step back.", z, up);
  else snprintf(m, sizeof(m), "Place relay %s here, then press D2.", z);
  sayToPi(k, m);
}

// Relay in the operator's hand (not yet D2): watch the beacon RSSI of the node behind it (B1: BASE, B2: B1).
// Weak for PLACE_N beacons -> PLACE every 15 s until D2; back above th + hysteresis -> stop. Upstream gone -> lost.
static void placeLoop(uint32_t now) {
  int mine = myCode();
  if (isCP || mine < 1 || mine >= NB_MAX || relayOff) return;
  static uint32_t lastSample = 0, lastSeenUsed[NB_MAX] = {0};
  static float ema[NB_MAX] = {0};
  static int strongN = 0;
  if (now - lastSample < 1000) return;
  lastSample = now;
  // the link behind this relay = the strongest live node closer to BASE (B1 down -> B2 watches BASE: heal)
  int best = -1;
  bool fresh = false, everHeard = false;
  for (int c = 0; c < mine; c++) {
    uint32_t seenAt = nb[c].seen;
    if (!seenAt) continue;
    everHeard = true;
    if (seenAt != lastSeenUsed[c]) {
      lastSeenUsed[c] = seenAt;
      int r = nb[c].rssi;
      ema[c] = ema[c] == 0 ? r : 0.6f * ema[c] + 0.4f * r;
      if (c == upCode) fresh = true;
    }
    if (now - seenAt < NB_ALIVE_MS && (best < 0 || ema[c] > ema[best])) best = c;
  }
  if (!everHeard) return;                                    // nothing behind us is on yet
  if (best >= 0) {
    if (best != upCode) fresh = true;
    upCode = best;
    upEma = ema[best];
  }
  if (placed) {                                              // brought back next to it: ready for a new run
    if (fresh) strongN = upEma > -40 ? strongN + 1 : 0;
    if (strongN >= 5) { placed = false; strongN = 0; screenDirty = true; Serial.println("# placement reset"); }
    return;
  }
  if (best >= 0) {
    if (upLost) { upLost = false; prompting = false; weakN = 0; screenDirty = true; }
    if (fresh) weakN = upEma < placeTh ? weakN + 1 : 0;
    if (prompting && upEma > placeTh + PLACE_HYST) { prompting = false; screenDirty = true; }   // walked back
    if (!prompting && weakN >= PLACE_N) { prompting = true; lastPlaceMsg = now - PLACE_REPEAT_MS; }
  } else if (!upLost) {                                      // no live node behind us at all
    upLost = prompting = true;
    lastPlaceMsg = now - PLACE_REPEAT_MS;
  }
  if (prompting && now - lastPlaceMsg >= PLACE_REPEAT_MS) {
    lastPlaceMsg = now;
    JsonDocument p;
    p["t"] = "PLACE";
    p["z"] = NODE_ID;
    p["up"] = NODE_NAME[upCode];
    p["r"] = (int)lround(upEma);
    if (upLost) p["lost"] = 1;
    originate(p, true);
    if (HAS_LINK_AP) placeSay(NODE_ID, NODE_NAME[upCode], upLost);
    Serial.printf("# place prompt up %s %d dBm%s\n", NODE_NAME[upCode], (int)lround(upEma), upLost ? " LOST" : "");
    screenDirty = true;
  }
}

// CMD {"to":<this node>,"c":...} from the dashboard (BASE serial -> down the chain)
static void runCmd(const char* c, int v) {
  if (strcmp(c, "off") == 0) relayOff = true;
  else if (strcmp(c, "on") == 0) relayOff = false;
  else if (strcmp(c, "lora") == 0 && HAS_LINK_AP) {
    loraOnly = true;
    JsonDocument d; d["t"] = "MODE"; d["lo"] = 1; toPi(d);
    sayToPi("lora", "Weak signal mode. LoRa only.");
  } else if (strcmp(c, "bulk") == 0 && HAS_LINK_AP) {
    loraOnly = false;
    JsonDocument d; d["t"] = "MODE"; d["lo"] = 0; toPi(d);
    sayToPi("bulk", "Full link restored.");
  } else if (strcmp(c, "th") == 0 && v < -5 && v > -100) placeTh = v;   // live tuning / bench test
  else { Serial.printf("# unknown cmd %s\n", c); return; }
  Serial.printf("# cmd %s ok\n", c);
  lastHeartbeat = millis() - HEARTBEAT_MS;      // report the new state right away
  screenDirty = true;
}

static void handlePacket(const Packet& pk) {
  JsonDocument doc;
  if (deserializeJson(doc, pk.data, pk.len)) { cntDrop++; return; }
  const char* id = doc["id"] | "";
  if (!id[0]) { cntDrop++; return; }

  // last hop = last entry of the route, or the source if the route is empty
  JsonArray rt = doc["rt"].as<JsonArray>();
  const char* hop = (rt && rt.size() > 0) ? (const char*)(rt[rt.size() - 1] | "?") : (doc["src"] | "?");
  if (strcmp(hop, NODE_ID) == 0 || strcmp(doc["src"] | "", NODE_ID) == 0) return;  // our own echo
  const char* t = doc["t"] | "";
  // local actions before the topology filter: B1's SL reaches B2 and BASE directly; a PLACE or MARK from the
  // relay in the operator's hand reaches B2, which is next to the Pi's speaker
  if (strcmp(id, localSeen) != 0) {
    bool act = true;
    if (strcmp(t, "SL") == 0 && !HAS_PORTAL) sniffStart(doc["pm"] | "");
    else if (strcmp(t, "PLACE") == 0 && HAS_LINK_AP)
      placeSay(doc["z"] | "?", doc["up"] | "?", (doc["lost"] | 0) != 0);
    else if (strcmp(t, "MARK") == 0 && HAS_LINK_AP) {
      char m[40], k[8];
      const char* z = doc["z"] | "?";
      snprintf(m, sizeof(m), "Relay %s placed.", z);
      snprintf(k, sizeof(k), "mark%c", z[0] ? z[strlen(z) - 1] : '?');
      sayToPi(k, m);
    } else act = false;
    if (act) { strncpy(localSeen, id, 4); localSeen[4] = 0; }
  }
  bool isAck = strcmp(t, "ACK") == 0;
  bool isDown = isAck || strcmp(t, "CMD") == 0;     // dashboard -> relays
  if (!acceptFrom(hop, isDown)) return;  // demo topology filter (silent)
  if (alreadySeen(id)) { cntDup++; return; }
  remember(id);
  if (strcmp(t, "CMD") == 0 && strcmp(doc["to"] | "", NODE_ID) == 0) { runCmd(doc["c"] | "", doc["v"] | 0); return; }

  cntRx++;
  strncpy(lastType, doc["t"] | "?", sizeof(lastType) - 1);
  strncpy(lastFrom, hop, sizeof(lastFrom) - 1);
  lastRssi = pk.rssi;
  screenDirty = true;

  if (isAck && HAS_PORTAL) {
    SosRec* r = findSos(doc["ref"] | "");
    if (r) {
      r->acked = true;
      strncpy(r->reply, doc["msg"] | "Help is on the way", sizeof(r->reply) - 1);
      r->reply[sizeof(r->reply) - 1] = 0;
      Serial.printf("# ack %s: %s\n", r->id, r->reply);
    }
  }

  if (isCP) {
    doc["q"] = pk.rssi;  // RSSI of the last hop, for the dashboard
    serializeJson(doc, Serial);
    Serial.println();
    return;
  }

  int ttl = doc["ttl"] | 0;
  if (ttl <= 0) return;
  doc["ttl"] = ttl - 1;
  if (!rt) rt = doc["rt"].to<JsonArray>();
  rt.add(NODE_ID);
  doc["q"] = pk.rssi;

  char buf[MAX_LEN + 1];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0 || n > MSG_MAX) { cntDrop++; return; }
  schedule(buf);
}

// ---------------------------------------------------------------------------
static void drawScreen() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(4, 4);
  tft.setTextSize(3);
  tft.setTextColor(relayOff ? ST77XX_RED : (isCP ? ST77XX_YELLOW : ST77XX_GREEN));
  tft.print(NODE_ID);
  tft.setTextSize(2);
  if (relayOff) { tft.setTextColor(ST77XX_RED); tft.print("  OFF"); }
  else if (loraOnly) { tft.setTextColor(ST77XX_ORANGE); tft.print("  LR-only"); }
  else { tft.setTextColor(demoTopology ? ST77XX_CYAN : ST77XX_MAGENTA); tft.print(demoTopology ? "  chain" : "  open"); }
  if (gaugeOk && batPct >= 0) {
    tft.setCursor(188, 8);
    tft.setTextColor(batPct < 20 ? ST77XX_RED : ST77XX_WHITE);
    tft.printf("%d%%", batPct);
  }
  if (relayOff) {                                  // the self-heal demo: make it obvious from across the room
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(4, 48);
    tft.print("RELAY OFF");
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(4, 90);
    tft.print("hold D1: back on");
    screenDirty = false;
    return;
  }
  tft.setTextColor(ST77XX_WHITE);
  if (prompting && !placed) {                      // relay in hand: drop it here
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.setCursor(4, 36);
    tft.print(upLost ? "STEP BACK" : "DROP HERE");
    tft.setTextSize(2);
    tft.setCursor(4, 66);
    tft.printf(upLost ? "lost %s" : "D2  %s %d dBm", NODE_NAME[upCode], (int)lround(upEma));
    tft.setTextColor(ST77XX_WHITE);
  } else {
    tft.setCursor(4, 40);
    tft.printf("rx %lu fwd %lu", (unsigned long)cntRx, (unsigned long)cntFwd);
    tft.setCursor(4, 62);
    tft.printf("dup %lu drop %lu", (unsigned long)cntDup, (unsigned long)cntDrop);
  }
  tft.setCursor(4, 90);
  tft.printf("%s<%s", lastType, lastFrom);
  tft.setCursor(4, 112);
  if (lastRssi) tft.printf("RSSI %d dBm", lastRssi);
  tft.setTextColor(ST77XX_YELLOW);
  static const char* const CODE_NAME[] = {"BASE", "B1", "B2", "Pi"};
  tft.setCursor(156, 90);
  if (bulkRx || bulkFwd) tft.printf("b>%s", isCP ? "USB" : (bulkVia < 0 ? "-" : CODE_NAME[bulkVia]));
  tft.setTextColor(ST77XX_WHITE);
  if (HAS_PORTAL) {
    tft.setCursor(170, 112);
    tft.setTextColor(ST77XX_ORANGE);
    tft.printf("AP %d", WiFi.softAPgetStationNum());
  }
  if (HAS_LORA) {
    tft.setCursor(170, 112);
    tft.setTextColor(loraOk ? ST77XX_GREEN : ST77XX_RED);
    tft.print(loraOk ? "LoRa" : "noLR");
  }
  screenDirty = false;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  isCP = strcmp(NODE_ID, "BASE") == 0;

#ifdef TFT_I2C_POWER
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif
#ifdef TFT_BACKLITE
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
#endif
  delay(10);
  tft.init(135, 240);
  tft.setRotation(3);
  Wire.begin();                                   // MAX17048 battery gauge (powered by TFT_I2C_POWER)
  Wire.beginTransmission(0x36);
  gaugeOk = Wire.endTransmission() == 0;

  pinMode(0, INPUT_PULLUP);    // D0: active low
  pinMode(1, INPUT_PULLDOWN);  // D1: active high
  pinMode(2, INPUT_PULLDOWN);  // D2: active high (MARK)

  if (HAS_PORTAL) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(PORTAL_SSID, nullptr, WIFI_CHANNEL);  // open network, same channel as ESP-NOW
  } else if (HAS_LINK_AP) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(LINK_SSID, LINK_PASS, WIFI_CHANNEL, 1 /*hidden*/, 2);  // the spotter Pi's bulk uplink
  } else {
    WiFi.mode(WIFI_STA);
  }
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    tft.fillScreen(ST77XX_RED);
    tft.setCursor(4, 4);
    tft.setTextSize(2);
    tft.print("ESP-NOW init failed");
    delay(3000);
    ESP.restart();             // was: halt forever (a relay must never stay dark until someone presses RESET)
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(bulkOnSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;  // current channel
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  if (HAS_LORA) {
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);  delay(10);
    digitalWrite(LORA_RST, HIGH); delay(10);
    pinMode(LORA_INT, INPUT);    // DIO0, polled from loop() (no interrupt, see RF95Polled)
    loraOk = rf95.init() && rf95.setFrequency(LORA_FREQ);
    Serial.printf("# lora %s\n", loraOk ? "ok" : "FAILED");
  }
  if (HAS_BLE) {
    BLEDevice::init("");
    bleScan = BLEDevice::getScan();
    bleScan->setAdvertisedDeviceCallbacks(new PhoneCb(), true);
    bleScan->setActiveScan(false);
    bleScan->setInterval(160);  // 100 ms
    bleScan->setWindow(48);     // 30 ms: leave airtime for ESP-NOW
  }

  if (HAS_PORTAL) portalBegin();
  if (HAS_LINK_AP) {
    linkUdp.listen(LINK_PORT);
    linkUdp.onPacket([](AsyncUDPPacket& pkt) {             // chunk from the Pi: same queue as ESP-NOW
      int len = pkt.length();
      const uint8_t* d = pkt.data();
      if (len < 10 || len > MAX_LEN || d[0] != BULK_MAGIC || d[1] != 1) return;
      piIp4 = (uint32_t)pkt.remoteIP();                      // where SAY / MODE go back to
      if (loraOnly || relayOff) { bulkDrop++; return; }      // weak-signal mode: alerts only (LoRa)
      bulkUdp++;
      portENTER_CRITICAL(&bulkMux);
      int next = (binHead + 1) % BULK_Q;
      if (next != binTail) {
        memcpy(bulkIn[binHead].d, d, len);
        bulkIn[binHead].d[2] = 3;                            // came from the Pi
        bulkIn[binHead].len = len;
        binHead = next;
      } else bulkDrop++;
      portEXIT_CRITICAL(&bulkMux);
    });
    Serial.printf("# link AP %s on %s udp %u\n", LINK_SSID, WiFi.softAPIP().toString().c_str(), LINK_PORT);
  }

  randomSeed(esp_random());
  lastHeartbeat = millis() - HEARTBEAT_MS + random(1000, 4000);  // first HB soon, nodes staggered
  drawScreen();
  // '#' = debug line, ignored by the dashboard; the board hub learns the node name from it
  Serial.printf("# {\"boot\":\"%s\",\"mac\":\"%s\"}\n", NODE_ID, WiFi.macAddress().c_str());
  Serial.printf("# reset reason %d\n", (int)esp_reset_reason());
  if (esp_reset_reason() == ESP_RST_SW && keepMagic == KEEP_MAGIC) {   // our own self-restart: carry on
    placed = keepPlaced; loraOnly = keepLora; placeTh = keepTh; wdWhy = keepWhy; wdGap = keepGap; wdStage = keepStage;
    Serial.printf("# self-restart (%s stuck %d ms, loop stage %d): placed %d lora-only %d th %d\n",
                  wdWhy == 1 ? "radio" : "loop", wdGap, wdStage, (int)placed, (int)loraOnly, placeTh);
  }
  keepMagic = 0;
  if (!isCP) {
    enableLoopWDT();   // relay: a loop stuck > 5 s reboots it (about 2 s) instead of hanging in the field
    Serial.printf("# loop wdt %s\n", esp_task_wdt_status(NULL) == ESP_OK ? "armed" : "NOT armed");
    lastTxDone = lastLoopAt = millis();
    TimerHandle_t wd = xTimerCreate("wd", pdMS_TO_TICKS(1000), pdTRUE, nullptr, wdTick);
    if (wd) xTimerStart(wd, 0);
  }
}

// BASE: the dashboard writes {"t":"ACK","ref":"<SOS id>","msg":"..."} lines to serial;
// BASE sends them back down the chain over ESP-NOW.
static void readSerialCommands() {
  static char line[MAX_LEN + 1];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (len < MAX_LEN) line[len++] = c; continue; }
    line[len] = 0;
    int n = len;
    len = 0;
    if (n == 0) continue;
    JsonDocument cmd;
    if (deserializeJson(cmd, line, n)) { Serial.println("# bad command (not JSON)"); continue; }
    if (strcmp(cmd["t"] | "", "CMD") == 0 && (cmd["to"] | "")[0] && (cmd["c"] | "")[0]) {
      JsonDocument o;                            // {"t":"CMD","to":"B2","c":"lora"}: down the chain like an ACK
      o["t"] = "CMD";
      o["to"] = cmd["to"] | "";
      o["c"] = cmd["c"] | "";
      if (cmd["v"].is<int>()) o["v"] = cmd["v"].as<int>();
      originate(o, true, true);
      Serial.printf("# cmd sent to=%s c=%s\n", (const char*)(cmd["to"] | ""), (const char*)(cmd["c"] | ""));
      continue;
    }
    if (strcmp(cmd["t"] | "", "ACK") != 0 || !(cmd["ref"] | "")[0]) { Serial.println("# ignored command"); continue; }
    JsonDocument ack;
    ack["t"] = "ACK";
    ack["ref"] = cmd["ref"] | "";
    String m = cmd["msg"] | "Help is on the way";
    if (m.length() > 40) m = m.substring(0, 40);
    ack["msg"] = m;
    originate(ack, true, true);
    Serial.printf("# ack sent ref=%s\n", (const char*)(cmd["ref"] | ""));
  }
}

void loop() {
  lastLoopAt = millis();                         // self-restart check (wdTick)
  loopStage = 1;
  if (isCP) readSerialCommands();
  bulkLoop();
  loopStage = 2;
  if (HAS_PORTAL) {
    dns.processNextRequest();
    web.handleClient();
  }
  // 1. drain received packets
  loopStage = 3;
  while (true) {
    Packet pk;
    bool have = false;
    portENTER_CRITICAL(&rxMux);
    if (rxTail != rxHead) {
      pk = rxQueue[rxTail];
      rxTail = (rxTail + 1) % QUEUE_SIZE;
      have = true;
    }
    portEXIT_CRITICAL(&rxMux);
    if (!have) break;
    handlePacket(pk);
  }

  // 1b. LoRa packets from the spotter go through the same relay logic
  loopStage = 4;
  if (loraOk && digitalRead(LORA_INT) == HIGH) rf95.poll();   // what the DIO0 interrupt used to do
  if (loraOk && rf95.available()) {
    Packet pk;
    uint8_t len = MAX_LEN;
    if (rf95.recv((uint8_t*)pk.data, &len)) {
      pk.data[len] = 0;
      pk.len = len;
      pk.rssi = rf95.lastRssi();
      handlePacket(pk);
    }
  }

  // 2. send due re-broadcasts
  uint32_t now = millis();

  // 2b. BLE: scan continuously in 5 s bursts, report a PHONE message every 20 s
  loopStage = 5;
  if (bleScan) {
    if (!bleScan->isScanning()) {
      bleScan->clearResults();
      bleScan->start(5, nullptr, false);
    }
    static uint32_t lastBle = 0;
    if (now - lastBle >= BLE_REPORT_MS) {
      lastBle = now;
      int n, best, ap, nr;
      portENTER_CRITICAL(&bleMux);
      n = bleCount; best = bleBest; ap = bleApple; nr = bleNear;
      bleCount = 0; bleApple = 0; bleBest = -127; bleNear = 0;
      portEXIT_CRITICAL(&bleMux);
      if (n > 0) {
        JsonDocument ph;
        ph["t"] = "PHONE";
        ph["n"] = n;
        ph["rssi"] = best;
        ph["ap"] = ap;
        ph["nr"] = nr;  // devices closer than NEAR_RSSI
        ph["z"] = NODE_ID;
        originate(ph);
      }
    }
  }
  for (Pending& p : pending) {
    if (p.used && (int32_t)(now - p.due) >= 0) {
      sendRaw(p.data, strlen(p.data));
      p.used = false;
      cntFwd++;
      screenDirty = true;
    }
  }

  // 3. heartbeat
  loopStage = 6;
  if (now - lastHeartbeat >= HEARTBEAT_MS) {  // BASE: printed to serial only (originate)
    lastHeartbeat = now;
    JsonDocument hb;
    hb["t"] = "HB";
    static bool firstHb = true;                              // first HB after boot: why we booted
    if (firstHb) {                                           // rr: 1 power-on or RESET key, 3 sw, 4 panic, 6/7 wdt, 9 brownout
      hb["rr"] = (int)esp_reset_reason();
      if (wdWhy) { hb["wd"] = wdWhy; hb["wg"] = wdGap; hb["ws"] = wdStage; }   // self-restart: why, ms, loop stage
      firstHb = false;
    }
    if (gaugeOk && batPct >= 0) { hb["bt"] = batPct; hb["bv"] = serialized(String(batV, 2)); }
    if (loraOnly) hb["lo"] = 1;
    originate(hb);
  }

  // 3b. battery gauge, every 10 s
  loopStage = 7;
  static uint32_t lastGauge = 0;
  if (gaugeOk && (now - lastGauge >= 10000 || lastGauge == 0)) {
    lastGauge = now;
    Wire.beginTransmission(0x36);
    Wire.write(0x02);                                        // VCELL (2 B, 78.125 uV/LSB), then SOC (2 B, 1/256 %)
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint16_t)0x36, (size_t)4, true) == 4) {
      uint16_t vc = Wire.read() << 8; vc |= Wire.read();
      uint8_t sh = Wire.read(), sl = Wire.read();
      int pct = min(100, sh + (sl >= 128 ? 1 : 0));
      batV = vc * 78.125e-6f;
      if (pct != batPct) screenDirty = true;
      batPct = pct;
    }
  }

  // 3c. relay placement prompt (relay in the operator's hand) and SOS phone listening
  loopStage = 8;
  placeLoop(now);
  loopStage = 9;
  sniffLoop(now);
  loopStage = 10;

  // 4. buttons (simple debounce)
  static uint32_t lastPress = 0;
  if (now - lastPress > 400) {
    if (digitalRead(0) == LOW) {
      lastPress = now;
      JsonDocument a;
      a["t"] = "PERSON";
      a["c"] = 88;
      a["th"] = 1;
      a["z"] = NODE_ID;
      originate(a);
      strncpy(lastType, "TEST", sizeof(lastType) - 1);
      strncpy(lastFrom, "me", sizeof(lastFrom) - 1);
      screenDirty = true;
    } else if (digitalRead(2) == HIGH) {
      lastPress = now;
      placed = true;               // this relay is in its spot: stop the placement prompt
      prompting = upLost = false;
      weakN = 0;
      JsonDocument m;
      m["t"] = "MARK";
      m["z"] = NODE_ID;
      originate(m, true);          // sent twice (same id): a lost frame gets a second chance
      if (HAS_LINK_AP) sayToPi("mark2", "Relay B2 placed.");
      strncpy(lastType, "MARKED", sizeof(lastType) - 1);
      strncpy(lastFrom, "me", sizeof(lastFrom) - 1);
      screenDirty = true;
    }
  }
  // D1: relay off / on (self-heal demo), held 1 s so a finger slipping off D2 next to it cannot switch the relay
  // off. Ignored on B2: it is the spotter's only gateway, so B2 off cuts the spotter off instead of healing.
  static uint32_t d1Since = 0;
  static bool d1Done = false;
  if (digitalRead(1) == HIGH) {
    if (!d1Since) d1Since = now | 1;
    if (!d1Done && now - d1Since >= 1000) {
      d1Done = true;
      if (!isCP && !HAS_LINK_AP) {
        relayOff = !relayOff;
        Serial.printf("# relay %s (D1)\n", relayOff ? "OFF" : "on");
        if (!relayOff) lastHeartbeat = now - HEARTBEAT_MS;
        screenDirty = true;
      } else Serial.println("# D1 ignored on this node");
    }
  } else {
    d1Since = 0;
    d1Done = false;
  }

  // 5. screen
  loopStage = 11;
  if (HAS_PORTAL) {  // show how many phones are on the rescue Wi-Fi
    static int lastSta = -1;
    int sta = WiFi.softAPgetStationNum();
    if (sta != lastSta) { lastSta = sta; screenDirty = true; }
  }
  static uint32_t lastDraw = 0;
  if (screenDirty && now - lastDraw > 250) {
    lastDraw = now;
    drawScreen();
  }
}
