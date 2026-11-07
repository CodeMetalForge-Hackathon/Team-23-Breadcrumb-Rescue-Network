// Breadcrumb Rescue Network: LoRa bridge for the spotter (SP1)
// Board: Adafruit Feather RP2040 RFM95 (#5714), 915 MHz. SOLDER THE ANTENNA BEFORE RUNNING.
//
// Pi5 (spotter) --USB serial, one JSON line per message--> this board --LoRa--> B2 gateway
//
// Serial in  : a JSON line, e.g. {"t":"PERSON","c":88,"th":1,"z":"B2"}
//              Missing "id"/"src"/"ttl"/"rt" are filled in (src = SP1). Max 250 bytes.
//              The plain line "test" sends a test PERSON alert.
// Serial out : {"sent":"<id>"} after each send, {"lora_rx":<json>,"rssi":-57} for anything heard
//              (ACKs coming back later), {"err":"..."} on bad input.
// Without a Pi: sends a HB every 15 s so B2 can show the LoRa link is alive.

#include <SPI.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>

#ifndef LORA_FREQ
#define LORA_FREQ 915.0
#endif
#ifndef LORA_TX_DBM
#define LORA_TX_DBM 13  // 5..23; 13 is plenty indoors
#endif

static const char* SRC = "SP1";
static const int DEFAULT_TTL = 4;
static const uint32_t HEARTBEAT_MS = 15000;
static const int MAX_LEN = 250;

RH_RF95 rf95(PIN_RFM_CS, PIN_RFM_DIO0);

static char line[MAX_LEN + 2];
static int lineLen = 0;
static uint32_t lastHb = 0;

static void randomId(char* out) {
  static const char hex[] = "0123456789abcdef";
  uint32_t r = rp2040.hwrand32();
  for (int i = 0; i < 4; i++) out[i] = hex[(r >> (i * 4)) & 0xF];
  out[4] = 0;
}

static void sendDoc(JsonDocument& doc) {
  char id[5];
  if (!doc["id"].is<const char*>()) { randomId(id); doc["id"] = id; }
  if (!doc["src"].is<const char*>()) doc["src"] = SRC;
  if (!doc["ttl"].is<int>()) doc["ttl"] = DEFAULT_TTL;
  if (!doc["rt"].is<JsonArray>()) doc["rt"].to<JsonArray>();
  char buf[MAX_LEN + 1];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0 || n > MAX_LEN) { Serial.println("{\"err\":\"too long\"}"); return; }
  digitalWrite(LED_BUILTIN, HIGH);
  rf95.send((const uint8_t*)buf, n);
  rf95.waitPacketSent();
  digitalWrite(LED_BUILTIN, LOW);
  Serial.printf("{\"sent\":\"%s\",\"len\":%u}\n", (const char*)(doc["id"] | ""), (unsigned)n);
}

static void handleLine(char* s) {
  while (*s == ' ' || *s == '\r') s++;
  if (!*s) return;
  JsonDocument doc;
  if (strcmp(s, "test") == 0) {
    doc["t"] = "PERSON"; doc["c"] = 88; doc["th"] = 1; doc["z"] = "B2";
  } else if (deserializeJson(doc, s)) {
    Serial.println("{\"err\":\"bad json\"}");
    return;
  }
  sendDoc(doc);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RFM_RST, OUTPUT);
  digitalWrite(PIN_RFM_RST, HIGH); delay(10);
  digitalWrite(PIN_RFM_RST, LOW);  delay(10);
  digitalWrite(PIN_RFM_RST, HIGH); delay(10);

  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);

  if (!rf95.init() || !rf95.setFrequency(LORA_FREQ)) {
    while (true) {  // fast blink = radio not found
      Serial.println("{\"err\":\"LoRa init failed\"}");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(150);
    }
  }
  rf95.setTxPower(LORA_TX_DBM, false);
  Serial.printf("{\"boot\":\"%s\",\"freq\":%.1f,\"dbm\":%d}\n", SRC, LORA_FREQ, LORA_TX_DBM);
  lastHb = millis() - HEARTBEAT_MS + 3000;
}

void loop() {
  // 1. serial lines from the Pi
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[lineLen] = 0;
      handleLine(line);
      lineLen = 0;
      lastHb = millis();  // Pi is talking, no need for filler heartbeats right now
    } else if (lineLen < MAX_LEN + 1) {
      line[lineLen++] = c;
    }
  }

  // 2. anything heard over LoRa (future ACKs)
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 1];
    uint8_t len = RH_RF95_MAX_MESSAGE_LEN;
    if (rf95.recv(buf, &len)) {
      buf[len] = 0;
      Serial.printf("{\"lora_rx\":%s,\"rssi\":%d}\n", (char*)buf, rf95.lastRssi());
    }
  }

  // 3. filler heartbeat
  if (millis() - lastHb >= HEARTBEAT_MS) {
    lastHb = millis();
    JsonDocument hb;
    hb["t"] = "HB";
    sendDoc(hb);
  }
}
