// ============================================================
//  BRIDGE ESP32 — Serial (USB/COM) ↔ ESP-NOW passthrough
//  Plug into laptop USB. Acts as transparent relay between
//  the HTML GUI (Web Serial) and the Car ESP32.
//
//  Car ESP32 MAC: set ROVER_MAC to the car's WiFi MAC address
//  Get it by running this on the car: WiFi.macAddress()
// ============================================================

#include <WiFi.h>
#include <esp_now.h>

// ── MAC ADDRESS OF CAR ESP32 ─────────────────────────────────
// Replace with actual MAC printed by car on Serial at boot
uint8_t roverMAC[] = {0x5C, 0x01, 0x3B, 0x6D, 0x91, 0x50};

// ── HAZARD CLUSTER CAPACITY (must match car_esp32.ino) ───────
#define HAZARD_MAX_CLUSTERS 16

// ── PACKET STRUCTURES (must match car_esp32.ino) ─────────────
struct __attribute__((packed)) TelemetryPacket {
  float   lat, lon;
  float   heading;
  float   bearingToTarget;
  float   distanceToTarget;
  float   speed;
  float   batteryV;
  float   hdop;
  uint8_t satellites;
  int8_t  rssi;
  uint8_t waypointIndex;
  uint8_t mode;
  uint8_t gpsFix;
  uint8_t commsLost;
  uint8_t isStuck;
  uint8_t  hazardCount;
  int8_t   hazardAngle[HAZARD_MAX_CLUSTERS];
  uint16_t hazardDist[HAZARD_MAX_CLUSTERS];
};

struct __attribute__((packed)) CommandPacket {
  double  targetLat, targetLon;
  float   arrivalRadius;
  uint8_t speedLimit;
  uint8_t mode;
  uint8_t manualDir;
  bool    headingLock;
  float   headingLockBearing;
  bool    autoAdvance;
  uint8_t waypointIndex;
  float   stuckThreshold;
  uint8_t hazardFOV;
  bool    hazardEnabled;
};

// ── SERIAL FRAMING ───────────────────────────────────────────
// GUI → Bridge: JSON line, e.g.:
//   {"type":"cmd","lat":...,"lon":...,"speed":150,...}\n
// Bridge → GUI: JSON line, e.g.:
//   {"type":"tele","lat":...,"lon":...,"bat":12.1,...}\n

void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Debug: print received vs expected size if mismatch
  if (len != sizeof(TelemetryPacket)) {
    Serial.print("{\"type\":\"err\",\"msg\":\"pkt size mismatch got ");
    Serial.print(len);
    Serial.print(" expected ");
    Serial.print(sizeof(TelemetryPacket));
    Serial.println("\"}");
    return;
  }

  TelemetryPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  // Format telemetry as JSON and send over Serial to laptop
  Serial.print("{\"type\":\"tele\"");
  Serial.print(",\"lat\":"  ); Serial.print(pkt.lat, 6);
  Serial.print(",\"lon\":"  ); Serial.print(pkt.lon, 6);
  Serial.print(",\"hdg\":"  ); Serial.print(pkt.heading, 1);
  Serial.print(",\"bear\":" ); Serial.print(pkt.bearingToTarget, 1);
  Serial.print(",\"dist\":" ); Serial.print(pkt.distanceToTarget, 1);
  Serial.print(",\"spd\":"  ); Serial.print(pkt.speed, 2);
  Serial.print(",\"bat\":"  ); Serial.print(pkt.batteryV, 2);
  Serial.print(",\"hdop\":" ); Serial.print(pkt.hdop, 2);
  Serial.print(",\"sats\":" ); Serial.print(pkt.satellites);
  Serial.print(",\"rssi\":" ); Serial.print(pkt.rssi);
  Serial.print(",\"wpIdx\":"); Serial.print(pkt.waypointIndex);
  Serial.print(",\"mode\":" ); Serial.print(pkt.mode);
  Serial.print(",\"fix\":"  ); Serial.print(pkt.gpsFix);
  Serial.print(",\"cls\":"  ); Serial.print(pkt.commsLost);
  Serial.print(",\"stuck\":"); Serial.print(pkt.isStuck);
  Serial.print(",\"haz\":[");
  for (uint8_t i = 0; i < pkt.hazardCount && i < HAZARD_MAX_CLUSTERS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print("{\"a\":"); Serial.print(pkt.hazardAngle[i]);
    Serial.print(",\"d\":");  Serial.print(pkt.hazardDist[i]);
    Serial.print("}");
  }
  Serial.print("]");
  Serial.println("}");
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  // Could track send success here if needed
}

// ── SERIAL COMMAND PARSER ────────────────────────────────────
// Reads a JSON line from Serial, parses key fields manually
// (avoids needing ArduinoJson library on bridge)

String serialBuf = "";

float  jsonFloat(String &s, const char* key) {
  String k = "\""; k += key; k += "\":";
  int i = s.indexOf(k);
  if (i < 0) return 0.0f;
  return s.substring(i + k.length()).toFloat();
}
double jsonDouble(String &s, const char* key) {
  String k = "\""; k += key; k += "\":";
  int i = s.indexOf(k);
  if (i < 0) return 0.0;
  return s.substring(i + k.length()).toDouble();
}
int jsonInt(String &s, const char* key) {
  String k = "\""; k += key; k += "\":";
  int i = s.indexOf(k);
  if (i < 0) return 0;
  return s.substring(i + k.length()).toInt();
}

void processCommand(String &line) {
  // {"type":"cmd","lat":...,"lon":...,"radius":7,"speed":150,
  //  "mode":0,"dir":1,"hlock":0,"hlbear":0,"adv":1,"wpIdx":0}

  CommandPacket cmd;
  cmd.targetLat         = jsonDouble(line, "lat");
  cmd.targetLon         = jsonDouble(line, "lon");
  cmd.arrivalRadius     = jsonFloat(line,  "radius");
  cmd.speedLimit        = (uint8_t)jsonInt(line, "speed");
  cmd.mode              = (uint8_t)jsonInt(line, "mode");
  cmd.manualDir         = (uint8_t)jsonInt(line, "dir");
  cmd.headingLock       = (bool)jsonInt(line, "hlock");
  cmd.headingLockBearing= jsonFloat(line, "hlbear");
  cmd.autoAdvance       = (bool)jsonInt(line, "adv");
  cmd.waypointIndex     = (uint8_t)jsonInt(line, "wpIdx");
  cmd.stuckThreshold    = jsonFloat(line, "stuckThr");
  cmd.hazardFOV         = (uint8_t)jsonInt(line, "hazFov");
  cmd.hazardEnabled     = (bool)jsonInt(line, "hazOn");

  esp_now_send(roverMAC, (uint8_t*)&cmd, sizeof(cmd));
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Print own MAC so user can copy it to car firmware
  WiFi.mode(WIFI_STA);
  Serial.print("Bridge MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("{\"type\":\"err\",\"msg\":\"ESP-NOW init failed\"}");
    return;
  }

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, roverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  Serial.println("{\"type\":\"ready\",\"msg\":\"Bridge online\"}");
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuf.trim();
      if (serialBuf.length() > 2) processCommand(serialBuf);
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }
}
