// Breadcrumb Rescue Network: ESP-NOW relay node
// Board: Adafruit Feather ESP32-S3 Reverse TFT (#5691)
//
// One sketch for every ESP32 node. Set NODE_ID below (or pass -DNODE_ID=\"B2\" at build time).
//   "B2" : gateway relay (LoRa receiver + BLE phone listener), "B1": relay
//   (flooding with de-dup, hop limit, random delay, heartbeat)
//   Demo chain with 3 ESP32s: SP1 --LoRa--> B2 --ESP-NOW--> B1 --ESP-NOW--> CP
//   (old names: B3 -> B2, B2 -> B1)
//   "CP"             : command post, also prints every new message to USB serial as one JSON line
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
  {"CP", {"B1", nullptr, nullptr}},
};

static const uint8_t WIFI_CHANNEL = 1;
static const int DEFAULT_TTL = 4;
static const uint32_t HEARTBEAT_MS = 15000;
static const uint32_t DELAY_MIN_MS = 50, DELAY_MAX_MS = 300;
static const int SEEN_SIZE = 50;
static const int QUEUE_SIZE = 16;
static const int MAX_LEN = 250;

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

// ---- BLE phone listener: count nearby phones (Apple / Google / Samsung / Microsoft adverts) ----
static const uint32_t BLE_REPORT_MS = 20000;
static const int BLE_MAX = 128;
static const int BLE_MIN_RSSI = -90;
static uint32_t bleSeen[BLE_MAX];
static int bleCount = 0, bleBest = -127;
static portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
static BLEScan* bleScan = nullptr;

class PhoneCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    int rssi = dev.getRSSI();
    if (rssi < BLE_MIN_RSSI || !dev.haveManufacturerData()) return;
    String m = dev.getManufacturerData();
    if (m.length() < 2) return;
    uint16_t company = (uint8_t)m[0] | ((uint8_t)m[1] << 8);
    // 0x004C Apple, 0x00E0 Google, 0x0075 Samsung, 0x0006 Microsoft
    if (company != 0x004C && company != 0x00E0 && company != 0x0075 && company != 0x0006) return;
    const uint8_t* a = (const uint8_t*)dev.getAddress().getNative();
    uint32_t h = ((uint32_t)a[2] << 24) | ((uint32_t)a[3] << 16) | ((uint32_t)a[4] << 8) | a[5];
    portENTER_CRITICAL(&bleMux);
    bool found = false;
    for (int i = 0; i < bleCount; i++) if (bleSeen[i] == h) { found = true; break; }
    if (!found && bleCount < BLE_MAX) bleSeen[bleCount++] = h;
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

static bool acceptFrom(const char* hop) {
  if (!demoTopology) return true;
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
static void originate(JsonDocument& doc) {
  char id[5];
  randomId(id);
  doc["id"] = id;
  doc["src"] = NODE_ID;
  doc["ttl"] = DEFAULT_TTL;
  doc["rt"].to<JsonArray>();
  char buf[MAX_LEN + 1];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0 || n > MAX_LEN) return;
  remember(id);
  if (isCP) Serial.println(buf);
  else sendRaw(buf, n);
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
  if (!acceptFrom(hop)) return;  // demo topology filter (silent)
  if (alreadySeen(id)) { cntDup++; return; }
  remember(id);

  cntRx++;
  strncpy(lastType, doc["t"] | "?", sizeof(lastType) - 1);
  strncpy(lastFrom, hop, sizeof(lastFrom) - 1);
  lastRssi = pk.rssi;
  screenDirty = true;

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
  if (n == 0 || n > MAX_LEN) { cntDrop++; return; }
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
  isCP = strcmp(NODE_ID, "CP") == 0;

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

  WiFi.mode(WIFI_STA);
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
    Serial.printf("{\"lora\":%s}\n", loraOk ? "true" : "false");
  }
  if (HAS_BLE) {
    BLEDevice::init("");
    bleScan = BLEDevice::getScan();
    bleScan->setAdvertisedDeviceCallbacks(new PhoneCb(), true);
    bleScan->setActiveScan(false);
    bleScan->setInterval(160);  // 100 ms
    bleScan->setWindow(48);     // 30 ms: leave airtime for ESP-NOW
  }

  randomSeed(esp_random());
  lastHeartbeat = millis() - HEARTBEAT_MS + random(1000, 4000);  // first HB soon, nodes staggered
  drawScreen();
  Serial.printf("{\"boot\":\"%s\",\"mac\":\"%s\"}\n", NODE_ID, WiFi.macAddress().c_str());
}

void loop() {
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
      int n, best;
      portENTER_CRITICAL(&bleMux);
      n = bleCount; best = bleBest;
      bleCount = 0; bleBest = -127;
      portEXIT_CRITICAL(&bleMux);
      if (n > 0) {
        JsonDocument ph;
        ph["t"] = "PHONE";
        ph["n"] = n;
        ph["rssi"] = best;
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
  if (!isCP && now - lastHeartbeat >= HEARTBEAT_MS) {
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
  static uint32_t lastDraw = 0;
  if (screenDirty && now - lastDraw > 250) {
    lastDraw = now;
    drawScreen();
  }
}
