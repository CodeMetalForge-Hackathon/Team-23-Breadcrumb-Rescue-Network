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
//          D1 toggles demo topology (only accept messages from the configured neighbour).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
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
#define HAS_BLE (__builtin_strcmp(NODE_ID, "B2") == 0)
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
struct SosRec { char id[5]; bool acked; char reply[48]; };
static SosRec sosLog[8];
static int sosNext = 0;
static const int LORA_CS = 10, LORA_RST = 11, LORA_INT = 9;
static const float LORA_FREQ = 915.0;
RH_RF95 rf95(LORA_CS, LORA_INT);
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
static uint32_t cntRx = 0, cntFwd = 0, cntDup = 0, cntDrop = 0;
static char lastType[10] = "-", lastFrom[8] = "-";
static int lastRssi = 0;
static bool screenDirty = true;

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

static bool acceptFrom(const char* hop, bool downlink) {
  if (!demoTopology) return true;
  if (downlink) {
    for (const Link& l : DOWNLINK) {
      if (strcmp(l.node, NODE_ID) != 0) continue;
      for (const char* a : l.acceptFrom)
        if (a && strcmp(a, hop) == 0) return true;
      return false;
    }
    return false;  // BASE (or unlisted node) never takes downlink traffic
  }
  for (const Link& l : TOPOLOGY) {
    if (strcmp(l.node, NODE_ID) != 0) continue;
    for (const char* a : l.acceptFrom)
      if (a && strcmp(a, hop) == 0) return true;
    return false;
  }
  return true;  // node not listed: accept everything
}

static void sendRaw(const char* s, int len) {
  esp_now_send(BROADCAST, (const uint8_t*)s, len);
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
  originate(doc, true);
  SosRec& rec = sosLog[sosNext];
  sosNext = (sosNext + 1) % 8;
  strncpy(rec.id, doc["id"] | "", 4); rec.id[4] = 0;
  rec.acked = false; rec.reply[0] = 0;
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

static void handlePacket(const Packet& pk) {
  JsonDocument doc;
  if (deserializeJson(doc, pk.data, pk.len)) { cntDrop++; return; }
  const char* id = doc["id"] | "";
  if (!id[0]) { cntDrop++; return; }

  // last hop = last entry of the route, or the source if the route is empty
  JsonArray rt = doc["rt"].as<JsonArray>();
  const char* hop = (rt && rt.size() > 0) ? (const char*)(rt[rt.size() - 1] | "?") : (doc["src"] | "?");
  if (strcmp(hop, NODE_ID) == 0 || strcmp(doc["src"] | "", NODE_ID) == 0) return;  // our own echo
  bool isAck = strcmp(doc["t"] | "", "ACK") == 0;
  if (!acceptFrom(hop, isAck)) return;  // demo topology filter (silent)
  if (alreadySeen(id)) { cntDup++; return; }
  remember(id);

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
  tft.setTextColor(isCP ? ST77XX_YELLOW : ST77XX_GREEN);
  tft.print(NODE_ID);
  tft.setTextSize(2);
  tft.setTextColor(demoTopology ? ST77XX_CYAN : ST77XX_MAGENTA);
  tft.print(demoTopology ? "  chain" : "  open");
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(4, 40);
  tft.printf("rx %lu fwd %lu", (unsigned long)cntRx, (unsigned long)cntFwd);
  tft.setCursor(4, 62);
  tft.printf("dup %lu drop %lu", (unsigned long)cntDup, (unsigned long)cntDrop);
  tft.setCursor(4, 90);
  tft.printf("%s<%s", lastType, lastFrom);
  tft.setCursor(4, 112);
  if (lastRssi) tft.printf("RSSI %d dBm", lastRssi);
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

  pinMode(0, INPUT_PULLUP);    // D0: active low
  pinMode(1, INPUT_PULLDOWN);  // D1: active high

  if (HAS_PORTAL) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(PORTAL_SSID, nullptr, WIFI_CHANNEL);  // open network, same channel as ESP-NOW
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
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

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

  randomSeed(esp_random());
  lastHeartbeat = millis() - HEARTBEAT_MS + random(1000, 4000);  // first HB soon, nodes staggered
  drawScreen();
  // '#' = debug line, ignored by the dashboard; the board hub learns the node name from it
  Serial.printf("# {\"boot\":\"%s\",\"mac\":\"%s\"}\n", NODE_ID, WiFi.macAddress().c_str());
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
  if (isCP) readSerialCommands();
  if (HAS_PORTAL) {
    dns.processNextRequest();
    web.handleClient();
  }
  // 1. drain received packets
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
  if (now - lastHeartbeat >= HEARTBEAT_MS) {  // BASE: printed to serial only (originate)
    lastHeartbeat = now;
    JsonDocument hb;
    hb["t"] = "HB";
    originate(hb);
  }

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
    } else if (digitalRead(1) == HIGH) {
      lastPress = now;
      demoTopology = !demoTopology;
      screenDirty = true;
    }
  }

  // 5. screen
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
