// ============================================================
//  CAR ESP32 — Autonomous GPS Rover with ESP-NOW telemetry
//  Hardware:
//    Motors   : BTS7960 x2 (existing wiring)
//    GPS      : Quectel L80-R on UART2 (RX=16, TX=17)
//    Compass  : LIS3MDL on I2C (SDA=21, SCL=22, addr=0x1C)
//    Battery  : 3S LiPo via voltage divider on GPIO36 (VP)
//    ESP-NOW  : Bridge MAC defined below
// ============================================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>

// ── MAC ADDRESS ─────────────────────────────────────────────
// Replace with the actual MAC of your Bridge ESP32
uint8_t bridgeMAC[] = {0x5C, 0x01, 0x3B, 0x64, 0x69, 0x28};

// ── MOTOR PINS (unchanged from original) ────────────────────
#define L_RPWM 25
#define L_LPWM 26
#define L_REN  27
#define L_LEN  14
#define R_RPWM 32
#define R_LPWM 33
#define R_REN  23
#define R_LEN  4

// ── PWM CHANNELS ────────────────────────────────────────────
const int chL_R = 0, chL_L = 1, chR_R = 2, chR_L = 3;
const int PWM_FREQ = 1000, PWM_RES = 8;

// ── GPS ─────────────────────────────────────────────────────
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial gpsSerial(2);
TinyGPSPlus    gps;

// ── LIDAR (D500 / STL-19P ranging core) — hazard avoidance ───
// Streams continuously at 230400 baud, no commands needed — RX only.
#define LIDAR_RX 18
HardwareSerial lidarSerial(1);

// Mounting offset: degrees to add so that 0° = straight ahead.
// Was 0 (sensor's own zero-mark was pointing at the rover's right side).
// Rotated 90° anticlockwise so true front now reports as bin 0.
#define LIDAR_MOUNT_OFFSET_DEG 270

#define HAZARD_STOP_MM 400   // forward motion blocked below this
#define HAZARD_SLOW_MM 900   // forward motion scaled down below this
#define HAZARD_CENTER_HALF_DEG 15  // ±deg checked for the stop/slow override
#define HAZARD_MAX_CLUSTERS 16     // ESP-NOW payload has plenty of headroom for this (packet stays well under the 250-byte cap)

uint8_t  hazardFOV     = 90;    // full field width in degrees (±45 default)
bool     hazardEnabled = false; // off by default until bench-tested against real hardware

uint16_t binDistMm[360];
unsigned long binTimeMs[360];
#define LIDAR_BIN_MAX_AGE_MS 400

struct HazardCluster { int8_t angle; uint16_t distMm; };
HazardCluster hazardClusters[HAZARD_MAX_CLUSTERS];
uint8_t hazardClusterCount = 0;

// ── COMPASS ─────────────────────────────────────────────────
#define LIS3MDL_ADDR 0x1C
#define EEPROM_SIZE  24
#define CAL_ADDR     0
Adafruit_LIS3MDL compass;
float offset_x = 0, offset_y = 0;
float cal_min_x, cal_max_x, cal_min_y, cal_max_y;
bool  calibrating = false;

// ── BATTERY ─────────────────────────────────────────────────
// 3S LiPo max 12.6V → 10kΩ + 1kΩ divider to VP pin
// Ratio = 1k/(10k+1k) = 0.0909 → at 12.6V gives 1.145V on VP (safe)
#define VBAT_PIN   36
#define VDIV_RATIO 0.09090f  // R2/(R1+R2)  — 10k + 1k divider
#define VBAT_LOW   10.5f
#define VBAT_CRIT  9.9f

// ── COMMS WATCHDOG ──────────────────────────────────────────
#define COMMS_TIMEOUT_MS 5000
unsigned long lastPacketMs = 0;
bool commsLost = false;
bool espNowStarted = false;

// ── MODE & CONTROL STATE ────────────────────────────────────
enum DriveMode { MODE_MANUAL, MODE_AUTO };
DriveMode driveMode = MODE_MANUAL;

// Manual
int  manualDir   = 0;   // bitmask: bit0=fwd, bit1=back, bit2=left, bit3=right
int  manualSpeed = 150; // 0–255
bool headingLockEnabled = false;
float headingLockTarget = -1;

// Auto
struct Waypoint { double lat; double lon; };
Waypoint targetWP = {0, 0};
bool     hasTarget      = false;
float    arrivalRadius  = 7.0f;   // metres
bool     autoAdvance    = true;
uint8_t  waypointIndex  = 0;

// ── STEERING PID (auto mode) ─────────────────────────────────
float pid_Kp = 1.8f, pid_Ki = 0.0f, pid_Kd = 0.4f;
float pid_integral = 0, pid_prevErr = 0;

// ── CURRENT TELEMETRY ───────────────────────────────────────
float  currentBearing = 0;
float  currentLat = 0, currentLon = 0;
float  batteryV   = 0;
float  hdop       = 99;
int    satellites = 0;
float  speed_mps  = 0;
int8_t lastRSSI = 0;

// Promiscuous callback to capture RSSI from incoming packets
void promiscuousRxCb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  lastRSSI = pkt->rx_ctrl.rssi;
}
bool   gpsFix     = false;

// ── STUCK DETECTION ─────────────────────────────────────────
float stuckThreshold    = 0.8f;
unsigned long stuckCheckMs  = 0;
unsigned long stuckSinceMs  = 0;
float lastCheckLat = 0, lastCheckLon = 0;
bool  isStuck = false;

// ── PACKET STRUCTURES ────────────────────────────────────────
// Sent to Bridge (telemetry)
struct __attribute__((packed)) TelemetryPacket {
  float  lat, lon;
  float  heading;
  float  bearingToTarget;
  float  distanceToTarget;
  float  speed;
  float  batteryV;
  float  hdop;
  uint8_t satellites;
  int8_t  rssi;
  uint8_t waypointIndex;
  uint8_t mode;         // 0=manual, 1=auto
  uint8_t gpsFix;
  uint8_t commsLost;
  uint8_t isStuck;
  uint8_t  hazardCount;
  int8_t   hazardAngle[HAZARD_MAX_CLUSTERS];
  uint16_t hazardDist[HAZARD_MAX_CLUSTERS];
};

// Received from Bridge (command)
struct __attribute__((packed)) CommandPacket {
  double  targetLat, targetLon;
  float   arrivalRadius;
  uint8_t speedLimit;
  uint8_t mode;          // 0=manual, 1=auto
  uint8_t manualDir;     // bitmask
  bool    headingLock;
  float   headingLockBearing;
  bool    autoAdvance;
  uint8_t waypointIndex;
  float   stuckThreshold;
  uint8_t hazardFOV;      // full field width in degrees
  bool    hazardEnabled;
};

// ── HELPERS ─────────────────────────────────────────────────
void setMotor(int rpwm, int lpwm, int val) {
  val = constrain(val, -255, 255);
  if (val >= 0) { ledcWrite(lpwm, 0); ledcWrite(rpwm, val); }
  else           { ledcWrite(rpwm, 0); ledcWrite(lpwm, -val); }
}
void stopAll() {
  ledcWrite(chL_R,0); ledcWrite(chL_L,0);
  ledcWrite(chR_R,0); ledcWrite(chR_L,0);
}
void idleMotors() { stopAll(); } // enables stay HIGH, no drive

float readBattery() {
  int raw = analogRead(VBAT_PIN);
  float vpin = (raw / 4095.0f) * 3.3f;
  return vpin / VDIV_RATIO;
}

// Heading from compass
float readHeading() {
  sensors_event_t ev;
  compass.getEvent(&ev);
  float cx = ev.magnetic.x - offset_x;
  float cy = ev.magnetic.y - offset_y;
  float b = atan2(cy, cx) * 180.0f / PI;
  if (b < 0) b += 360.0f;
  return b;
}

// Great-circle bearing from current pos to target
float bearingTo(double fromLat, double fromLon, double toLat, double toLon) {
  double dLon = radians(toLon - fromLon);
  double lat1 = radians(fromLat), lat2 = radians(toLat);
  double x = sin(dLon) * cos(lat2);
  double y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  float b = degrees(atan2(x, y));
  if (b < 0) b += 360.0f;
  return b;
}

// Haversine distance in metres
float distanceTo(double fromLat, double fromLon, double toLat, double toLon) {
  const double R = 6371000.0;
  double dLat = radians(toLat - fromLat);
  double dLon = radians(toLon - fromLon);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(fromLat))*cos(radians(toLat))*sin(dLon/2)*sin(dLon/2);
  return (float)(R * 2 * atan2(sqrt(a), sqrt(1-a)));
}

// Shortest signed angle difference (-180 to +180)
float angleDiff(float target, float current) {
  float d = target - current;
  while (d >  180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

// ── LIDAR PARSING (D500 / STL-19P packet format) ─────────────
// 0x54 header, ver_len (low 5 bits = point count), speed, start_angle,
// N x (2B distance mm + 1B intensity), end_angle, timestamp, crc8
static const uint8_t LIDAR_CRC_TABLE[256] = {
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
};

uint8_t lidarCrc8(const uint8_t* buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc = LIDAR_CRC_TABLE[(crc ^ buf[i]) & 0xff];
  return crc;
}

uint8_t lidarFrameBuf[128];
uint8_t lidarFrameIdx = 0;
uint8_t lidarExpectedLen = 0;

void storeLidarPoint(float angleDeg, uint16_t distMm) {
  if (distMm == 0) return; // no return
  float a = angleDeg - LIDAR_MOUNT_OFFSET_DEG;
  while (a < 0)   a += 360.0f;
  while (a >= 360) a -= 360.0f;
  int bin = ((int)a) % 360;
  binDistMm[bin] = distMm;
  binTimeMs[bin] = millis();
}

void parseLidarFrame(const uint8_t* frame, uint8_t numPoints) {
  uint16_t speed      = frame[2] | (frame[3] << 8);          // deg/s (unused directly)
  (void)speed;
  float startAngle    = (frame[4] | (frame[5] << 8)) / 100.0f;
  uint8_t offset = 6;
  uint16_t dist[31]; uint8_t inten[31];
  for (uint8_t i = 0; i < numPoints; i++) {
    dist[i]  = frame[offset] | (frame[offset+1] << 8);
    inten[i] = frame[offset+2];
    offset += 3;
  }
  float endAngle = (frame[offset] | (frame[offset+1] << 8)) / 100.0f; offset += 2;
  // timestamp at frame[offset..offset+1], not needed here

  float span = endAngle - startAngle;
  if (span < 0) span += 360.0f;
  float step = (numPoints > 1) ? span / (numPoints - 1) : 0;

  for (uint8_t i = 0; i < numPoints; i++) {
    float a = startAngle + step * i;
    if (a >= 360.0f) a -= 360.0f;
    storeLidarPoint(a, dist[i]);
  }
}

void processLidarByte(uint8_t b) {
  if (lidarFrameIdx == 0) {
    if (b != 0x54) return;
    lidarFrameBuf[0] = b; lidarFrameIdx = 1; lidarExpectedLen = 0;
    return;
  }
  if (lidarFrameIdx == 1) {
    lidarFrameBuf[1] = b;
    uint8_t numPoints = b & 0x1F;
    lidarExpectedLen = 1 + 1 + 2 + 2 + numPoints * 3 + 2 + 2 + 1;
    if (lidarExpectedLen > sizeof(lidarFrameBuf)) { lidarFrameIdx = 0; return; }
    lidarFrameIdx = 2;
    return;
  }
  lidarFrameBuf[lidarFrameIdx++] = b;
  if (lidarFrameIdx >= lidarExpectedLen) {
    uint8_t crcCalc = lidarCrc8(lidarFrameBuf, lidarExpectedLen - 1);
    if (crcCalc == lidarFrameBuf[lidarExpectedLen - 1]) {
      parseLidarFrame(lidarFrameBuf, lidarFrameBuf[1] & 0x1F);
    }
    lidarFrameIdx = 0;
  }
}

void readLidar() {
  while (lidarSerial.available()) processLidarByte((uint8_t)lidarSerial.read());
}

// Groups adjacent valid bins within ±hazardFOV/2 into up to 8 clusters,
// keeping the nearest distance in each. Cheap O(FOV) pass, run at telemetry rate.
void buildHazardClusters() {
  hazardClusterCount = 0;
  unsigned long now = millis();
  int halfFov = hazardFOV / 2;

  bool inCluster = false;
  int clusterStartAngle = 0;
  uint16_t clusterMinDist = 0;
  uint16_t lastDist = 0;

  for (int a = -halfFov; a <= halfFov; a++) {
    int bin = ((a % 360) + 360) % 360;
    bool valid = (binDistMm[bin] > 0) && (now - binTimeMs[bin] < LIDAR_BIN_MAX_AGE_MS);
    uint16_t d = valid ? binDistMm[bin] : 0;

    if (valid) {
      if (!inCluster) {
        inCluster = true;
        clusterStartAngle = a;
        clusterMinDist = d;
      } else if (abs((int)d - (int)lastDist) < 300) {
        if (d < clusterMinDist) clusterMinDist = d;
      } else {
        if (hazardClusterCount < HAZARD_MAX_CLUSTERS) {
          hazardClusters[hazardClusterCount].angle  = (int8_t)((clusterStartAngle + (a - 1)) / 2);
          hazardClusters[hazardClusterCount].distMm = clusterMinDist;
          hazardClusterCount++;
        }
        clusterStartAngle = a;
        clusterMinDist = d;
      }
      lastDist = d;
    } else if (inCluster) {
      if (hazardClusterCount < HAZARD_MAX_CLUSTERS) {
        hazardClusters[hazardClusterCount].angle  = (int8_t)((clusterStartAngle + (a - 1)) / 2);
        hazardClusters[hazardClusterCount].distMm = clusterMinDist;
        hazardClusterCount++;
      }
      inCluster = false;
    }
    if (hazardClusterCount >= HAZARD_MAX_CLUSTERS) return;
  }
  if (inCluster && hazardClusterCount < HAZARD_MAX_CLUSTERS) {
    hazardClusters[hazardClusterCount].angle  = (int8_t)((clusterStartAngle + halfFov) / 2);
    hazardClusters[hazardClusterCount].distMm = clusterMinDist;
    hazardClusterCount++;
  }
}

// Nearest obstacle in the narrow center cone dead ahead — used for the drive override
float hazardMinDistCenterMm() {
  float minD = 99999;
  unsigned long now = millis();
  for (int a = -HAZARD_CENTER_HALF_DEG; a <= HAZARD_CENTER_HALF_DEG; a++) {
    int bin = ((a % 360) + 360) % 360;
    if (binDistMm[bin] > 0 && (now - binTimeMs[bin] < LIDAR_BIN_MAX_AGE_MS)) {
      if (binDistMm[bin] < minD) minD = binDistMm[bin];
    }
  }
  return minD;
}

// Blocks/scales forward motion only — steering and reverse stay under full control
void applyHazardOverride(int &L, int &R) {
  if (!hazardEnabled) return;
  float hz = hazardMinDistCenterMm();
  if (hz < HAZARD_STOP_MM) {
    if (L > 0) L = 0;
    if (R > 0) R = 0;
  } else if (hz < HAZARD_SLOW_MM) {
    float scale = (hz - HAZARD_STOP_MM) / (float)(HAZARD_SLOW_MM - HAZARD_STOP_MM);
    if (L > 0) L = (int)(L * scale);
    if (R > 0) R = (int)(R * scale);
  }
}

// ── EEPROM ──────────────────────────────────────────────────
void saveCalibration() {
  EEPROM.put(CAL_ADDR,     offset_x);
  EEPROM.put(CAL_ADDR + 4, offset_y);
  EEPROM.commit();
}
void loadCalibration() {
  EEPROM.get(CAL_ADDR,     offset_x);
  EEPROM.get(CAL_ADDR + 4, offset_y);
  if (isnan(offset_x) || isnan(offset_y)) { offset_x = 0; offset_y = 0; }
}

// ── ESP-NOW CALLBACKS ───────────────────────────────────────
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != sizeof(CommandPacket)) return;
  lastPacketMs = millis();
  commsLost = false;

  CommandPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));

  driveMode    = (cmd.mode == 1) ? MODE_AUTO : MODE_MANUAL;
  manualDir    = cmd.manualDir;
  manualSpeed  = cmd.speedLimit;
  arrivalRadius= cmd.arrivalRadius;
  autoAdvance  = cmd.autoAdvance;
  waypointIndex= cmd.waypointIndex;
  if (cmd.stuckThreshold > 0) stuckThreshold = cmd.stuckThreshold;
  headingLockEnabled = cmd.headingLock;
  if (cmd.headingLock) headingLockTarget = cmd.headingLockBearing;
  if (cmd.hazardFOV > 0) hazardFOV = cmd.hazardFOV;
  hazardEnabled = cmd.hazardEnabled;

  if (driveMode == MODE_AUTO && (cmd.targetLat != 0 || cmd.targetLon != 0)) {
    targetWP.lat = cmd.targetLat;
    targetWP.lon = cmd.targetLon;
    hasTarget = true;
    pid_integral = 0; pid_prevErr = 0;
  }
}

void sendTelemetry() {
  float dist = 0, bear = 0;
  if (hasTarget && gpsFix) {
    dist = distanceTo(currentLat, currentLon, targetWP.lat, targetWP.lon);
    bear = bearingTo(currentLat, currentLon, targetWP.lat, targetWP.lon);
  }

  TelemetryPacket pkt;
  pkt.lat             = currentLat;
  pkt.lon             = currentLon;
  pkt.heading         = currentBearing;
  pkt.bearingToTarget = bear;
  pkt.distanceToTarget= dist;
  pkt.speed           = speed_mps;
  pkt.batteryV        = batteryV;
  pkt.hdop            = hdop;
  pkt.satellites      = (uint8_t)satellites;
  pkt.rssi            = lastRSSI;
  pkt.waypointIndex   = waypointIndex;
  pkt.mode            = (driveMode == MODE_AUTO) ? 1 : 0;
  pkt.gpsFix          = gpsFix ? 1 : 0;
  pkt.commsLost       = commsLost ? 1 : 0;
  pkt.isStuck         = isStuck ? 1 : 0;

  pkt.hazardCount = hazardClusterCount;
  for (uint8_t i = 0; i < HAZARD_MAX_CLUSTERS; i++) {
    pkt.hazardAngle[i] = (i < hazardClusterCount) ? hazardClusters[i].angle  : 0;
    pkt.hazardDist[i]  = (i < hazardClusterCount) ? hazardClusters[i].distMm : 0;
  }

  esp_now_send(bridgeMAC, (uint8_t*)&pkt, sizeof(pkt));
}

// ── DRIVE LOGIC ──────────────────────────────────────────────
void driveManual() {
  bool fwd   = manualDir & 0x01;
  bool back  = manualDir & 0x02;
  bool left  = manualDir & 0x04;
  bool right = manualDir & 0x08;

  int spd = constrain(manualSpeed, 0, 255);

  // Battery limiting
  if (batteryV > 3.0f && batteryV < VBAT_LOW) spd = spd * 50 / 100;

  int L = 0, R = 0;

  if (fwd || back) {
    int base = fwd ? spd : -spd;
    // Bias: pressing left reduces right motor, pressing right reduces left motor
    // Pressing only left or right spins in place
    if (left && !right) {
      L = -base; R = base;       // spin left in place if no fwd/back... 
      // But fwd is true here, so bias:
      if (fwd || back) { L = base / 2; R = base; }  // gentle bias
      // Full spin if ONLY left
      if (!fwd && !back) { L = -spd; R = spd; }
    } else if (right && !left) {
      if (fwd || back) { L = base; R = base / 2; }
      if (!fwd && !back) { L = spd; R = -spd; }
    } else {
      L = base; R = base;
    }
  } else if (left && !right) {
    L = -spd; R = spd;   // spin left
  } else if (right && !left) {
    L = spd; R = -spd;   // spin right
  }

  // Heading lock overrides when going straight (no left/right pressed)
  if (headingLockEnabled && headingLockTarget >= 0 && (fwd || back) && !left && !right) {
    float err  = angleDiff(headingLockTarget, currentBearing);
    float corr = constrain(err * 1.5f, -80, 80);
    L = constrain(L - (int)corr, -255, 255);
    R = constrain(R + (int)corr, -255, 255);
  }

  applyHazardOverride(L, R);

  setMotor(chL_R, chL_L, L);
  setMotor(chR_R, chR_L, R);
}

void driveAuto() {
  if (!hasTarget || !gpsFix) { idleMotors(); return; }

  float dist = distanceTo(currentLat, currentLon, targetWP.lat, targetWP.lon);

  // Scale arrival radius by HDOP
  float effectiveRadius = arrivalRadius;
  if (hdop > 2.0f) effectiveRadius = arrivalRadius + (hdop - 2.0f) * 2.0f;
  effectiveRadius = constrain(effectiveRadius, arrivalRadius, 20.0f);

  if (dist < effectiveRadius) {
    // Arrived
    idleMotors();
    if (autoAdvance) {
      waypointIndex++;
      hasTarget = false;   // Bridge will send next waypoint
    }
    return;
  }

  // PID steering on bearing error
  float targetBear = bearingTo(currentLat, currentLon, targetWP.lat, targetWP.lon);
  float err = angleDiff(targetBear, currentBearing);

  pid_integral += err;
  pid_integral  = constrain(pid_integral, -200, 200);
  float deriv   = err - pid_prevErr;
  pid_prevErr   = err;

  float steer = pid_Kp * err + pid_Ki * pid_integral + pid_Kd * deriv;
  steer = constrain(steer, -255, 255);

  int baseSpeed = constrain(manualSpeed, 0, 255);
  if (batteryV > 3.0f && batteryV < VBAT_LOW) baseSpeed = baseSpeed * 50 / 100;

  // Reduce speed when far off-heading
  float headingFactor = 1.0f - (abs(err) / 180.0f) * 0.5f;
  int fwdSpeed = (int)(baseSpeed * headingFactor);

  int L = constrain(fwdSpeed + (int)steer, -255, 255);
  int R = constrain(fwdSpeed - (int)steer, -255, 255);

  applyHazardOverride(L, R);

  setMotor(chL_R, chL_L, L);
  setMotor(chR_R, chR_L, R);
}

// ── STUCK DETECTION ─────────────────────────────────────────
const unsigned long STUCK_CONFIRM_MS = 4000;  // must be stationary for 4s before flagging

void updateStuckDetection() {
  if (!gpsFix) { isStuck = false; return; }

  // Only check when motors are commanded to run
  bool motorsActive = false;
  if (driveMode == MODE_MANUAL && manualDir != 0) motorsActive = true;
  if (driveMode == MODE_AUTO   && hasTarget && !commsLost) motorsActive = true;
  if (!motorsActive) { isStuck = false; stuckSinceMs = 0; return; }

  unsigned long now = millis();
  if (now - stuckCheckMs < 2000) return;  // check every 2s
  stuckCheckMs = now;

  float moved = distanceTo(lastCheckLat, lastCheckLon, currentLat, currentLon);

  if (lastCheckLat == 0 && lastCheckLon == 0) {
    // First check — just record position
    lastCheckLat = currentLat; lastCheckLon = currentLon;
    return;
  }

  if (moved < stuckThreshold) {
    if (stuckSinceMs == 0) stuckSinceMs = now;
    if (now - stuckSinceMs > STUCK_CONFIRM_MS) isStuck = true;
  } else {
    isStuck = false; stuckSinceMs = 0;
    lastCheckLat = currentLat; lastCheckLon = currentLon;
  }
}
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(21, 22);

  // Motor pins
  pinMode(L_REN, OUTPUT); pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT); pinMode(R_LEN, OUTPUT);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);

  ledcSetup(chL_R, PWM_FREQ, PWM_RES); ledcAttachPin(L_RPWM, chL_R);
  ledcSetup(chL_L, PWM_FREQ, PWM_RES); ledcAttachPin(L_LPWM, chL_L);
  ledcSetup(chR_R, PWM_FREQ, PWM_RES); ledcAttachPin(R_RPWM, chR_R);
  ledcSetup(chR_L, PWM_FREQ, PWM_RES); ledcAttachPin(R_LPWM, chR_L);
  stopAll();

  // Battery pin
  pinMode(VBAT_PIN, INPUT);

  // Compass
  if (!compass.begin_I2C(LIS3MDL_ADDR)) {
    Serial.println("LIS3MDL not found!");
  } else {
    compass.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    compass.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    compass.setDataRate(LIS3MDL_DATARATE_10_HZ);
    compass.setRange(LIS3MDL_RANGE_4_GAUSS);
  }
  loadCalibration();

  // GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // LiDAR (RX only — sensor streams continuously, no config needed)
  lidarSerial.begin(230400, SERIAL_8N1, LIDAR_RX, -1);
  for (int i = 0; i < 360; i++) { binDistMm[i] = 0; binTimeMs[i] = 0; }

  // Start ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  } else {
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, bridgeMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    esp_now_register_recv_cb(onDataRecv);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuousRxCb);
    espNowStarted = true;
    Serial.println("ESP-NOW started");
  }
  lastPacketMs = millis();
}



// ── LOOP ─────────────────────────────────────────────────────
unsigned long lastTelemetryMs = 0;
unsigned long lastBattMs      = 0;
unsigned long bootMs          = millis();

void loop() {
  unsigned long now = millis();

  // LiDAR
  readLidar();
  buildHazardClusters();

  // Feed GPS
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    gpsFix     = true;
  } else { gpsFix = false; }
  if (gps.hdop.isValid())      hdop       = gps.hdop.hdop();
  if (gps.satellites.isValid()) satellites = gps.satellites.value();
  if (gps.speed.isValid()) {
    float raw = gps.speed.mps();
    speed_mps = (raw < 0.4f) ? 0.0f : raw;  // filter GPS noise when stationary
  }

  // Compass
  currentBearing = readHeading();
  if (calibrating) {
    sensors_event_t ev; compass.getEvent(&ev);
    if (ev.magnetic.x < cal_min_x) cal_min_x = ev.magnetic.x;
    if (ev.magnetic.x > cal_max_x) cal_max_x = ev.magnetic.x;
    if (ev.magnetic.y < cal_min_y) cal_min_y = ev.magnetic.y;
    if (ev.magnetic.y > cal_max_y) cal_max_y = ev.magnetic.y;
  }

  // Battery (every 5s)
  if (now - lastBattMs > 5000) { batteryV = readBattery(); lastBattMs = now; }

  // Diag AP — serve web page but allow manual driving
  if (!espNowStarted) return;

  // Comms watchdog
  if (now - lastPacketMs > COMMS_TIMEOUT_MS) {
    commsLost = true;
    idleMotors();
  } else {
    if (batteryV > 3.0f && batteryV < VBAT_CRIT) {
      idleMotors();
    } else {
      updateStuckDetection();
      if (driveMode == MODE_MANUAL) driveManual();
      else                          driveAuto();
    }
  }

  // Telemetry: send immediately after receiving a command (done in onDataRecv)
  // Heartbeat: send telemetry every 500ms regardless, so GUI knows car is alive
  if (now - lastTelemetryMs > 500) {
    sendTelemetry();
    lastTelemetryMs = now;
  }
}
