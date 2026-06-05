/*
  ============================================================
  PROJECT:   ESP32 Boat Receiver – RZ7886 Dual H-Bridge Version
  AUTHOR:    David Gilmour
  DATE:      October 2025
  BOARD:     ESP32-WROOM-32U-N4
  ============================================================

  DESCRIPTION:
  ------------------------------------------------------------
  Runs on the boat-side ESP32 and drives the RZ7886 dual motor driver
  (two inputs per channel). Each motor uses:
     • FI – Forward input  (PWM when moving forward)
     • BI – Backward input (PWM when moving reverse)

  Receives joystick commands ("cmdX,cmdY") via ESP-NOW, drives motors
  with correct FI/BI logic, sends telemetry (RSSI & battery voltage),
  and stops motors on timeout or undervoltage.

  ============================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
extern "C" {
  #include "esp_wifi_types.h"
}

// ===== Pin definitions (two inputs per channel) =====
const int motorAFI = 19;  // Left motor forward input
const int motorABI = 18;  // Left motor backward input
const int motorBFI = 5;   // Right motor forward input
const int motorBBI = 4;   // Right motor backward input
const int VP_PIN   = 36;  // Battery divider (20k:10k)

// --- Optional enable pins (tie HIGH if not present) ---
const int ENA = -1;  // set to pin number if your board has ENA
const int ENB = -1;  // set to pin number if your board has ENB

// ===== PWM config =====
const int pwmFreq = 1000;       // 1 kHz gives strong torque, low hiss
const int pwmResolution = 8;    // 8-bit (0-255)
const int maxDuty = 254;        // ~99.6 %

// ===== Communication =====
const uint8_t TRANSMITTER_MAC[6] = {0xCC,0xDB,0xA7,0x98,0x13,0xD0};
const uint8_t WIFI_CHANNEL = 1;
const unsigned long COMM_TIMEOUT_MS = 300;

// ===== Motor polarity (flip if reversed) =====
const bool INVERT_A = false;
const bool INVERT_B = false;

// ===== Battery settings =====
const float ADC_REF_V = 3.3f;
const float DIVIDER_GAIN = 3.0f;
const float MIN_SAFE_VOLTAGE = 6.4f;
const float MAX_SANE_VOLTAGE = 15.0f;

// ===== Runtime state =====
volatile unsigned long lastRxMs = 0;
volatile int8_t lastPeerRSSI = 127;

// ===== Helper functions =====
static inline void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

static inline void safeDriveOff() {
  stopMotors();
  digitalWrite(motorAFI, LOW);
  digitalWrite(motorABI, LOW);
  digitalWrite(motorBFI, LOW);
  digitalWrite(motorBBI, LOW);
}

// Drive one motor per RZ7886 truth table
static inline void driveMotorRZ7886(int16_t cmd,
                                    int pinFI, int pinBI,
                                    int chFI,  int chBI,
                                    bool invert) {
  if (invert) cmd = -cmd;
  int duty = abs(cmd);
  duty = constrain(duty, 0, 255);
  duty = map(duty, 0, 255, 0, maxDuty);

  if (cmd > 0) {
    // Forward: FI = PWM, BI = LOW
    ledcWrite(chBI, 0);
    digitalWrite(pinBI, LOW);
    ledcWrite(chFI, duty);
  } else if (cmd < 0) {
    // Reverse: BI = PWM, FI = LOW
    ledcWrite(chFI, 0);
    digitalWrite(pinFI, LOW);
    ledcWrite(chBI, duty);
  } else {
    // Stop / coast: both LOW
    ledcWrite(chFI, 0);
    ledcWrite(chBI, 0);
    digitalWrite(pinFI, LOW);
    digitalWrite(pinBI, LOW);
    // To brake instead, uncomment:
    // digitalWrite(pinFI, HIGH);
    // digitalWrite(pinBI, HIGH);
  }
}

// Read battery voltage (mV)
static inline uint16_t readBattery_mV(int pin) {
  analogSetPinAttenuation(pin, ADC_11db);
  int raw = analogRead(pin);
  float v_pin = (raw / 4095.0f) * ADC_REF_V;
  float v_bat = v_pin * DIVIDER_GAIN;
  if (v_bat < 0.5f || v_bat > 20.0f) return 0;
  return (uint16_t)lroundf(v_bat * 1000.0f);
}

// Parse "cmdX,cmdY"
static bool parseCSVCmds(const char* buf, int16_t& outX, int16_t& outY) {
  int x=0, y=0;
  if (sscanf(buf, "%d,%d", &x, &y) == 2) {
    outX = constrain(x, -255, 255);
    outY = constrain(y, -255, 255);
    return true;
  }
  return false;
}

// ===== RSSI capture =====
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  const wifi_promiscuous_pkt_t *ppkt = (const wifi_promiscuous_pkt_t *)buf;
  const uint8_t *payload = ppkt->payload;
  const uint8_t *addr2 = payload + 10;
  if (memcmp(addr2, TRANSMITTER_MAC, 6) == 0) {
    lastPeerRSSI = ppkt->rx_ctrl.rssi;
    esp_wifi_set_promiscuous(false);
  }
}

// ===== ESP-NOW callbacks =====
void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.printf("Send → %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataRecv(const uint8_t*, const uint8_t* data, int len) {
  static char buf[32];
  if (len <= 0) return;
  if (len >= (int)sizeof(buf)) len = sizeof(buf)-1;
  memcpy(buf, data, len);
  buf[len] = '\0';
  Serial.printf("RX CSV: '%s'\n", buf);

  int16_t cmdX = 0, cmdY = 0;
  if (!parseCSVCmds(buf, cmdX, cmdY)) return;
  lastRxMs = millis();

  // Dead zone
  if (abs(cmdX) < 3) cmdX = 0;
  if (abs(cmdY) < 3) cmdY = 0;

  // Battery read
  uint16_t batt_mV = readBattery_mV(VP_PIN);
  float batt_V = batt_mV / 1000.0f;

  // Drive logic
  if (batt_V < 1.0f || batt_V > MAX_SANE_VOLTAGE) {
    safeDriveOff();
    Serial.println("Invalid battery reading – motors off");
  } else if (batt_V < MIN_SAFE_VOLTAGE) {
    cmdX /= 3; cmdY /= 3;
    driveMotorRZ7886(cmdY, motorAFI, motorABI, 0, 1, INVERT_A);
    driveMotorRZ7886(cmdX, motorBFI, motorBBI, 2, 3, INVERT_B);
    Serial.println("Low voltage: limited drive");
  } else {
    driveMotorRZ7886(cmdY, motorAFI, motorABI, 0, 1, INVERT_A);
    driveMotorRZ7886(cmdX, motorBFI, motorBBI, 2, 3, INVERT_B);
  }

  // Telemetry back
  char out[24];
  snprintf(out, sizeof(out), "%d,%u", (int)lastPeerRSSI, (unsigned)batt_mV);
  esp_now_send(TRANSMITTER_MAC, (uint8_t*)out, strlen(out));
  Serial.printf("Telemetry sent: RSSI=%d dBm, Battery=%u mV\n\n",
                (int)lastPeerRSSI, (unsigned)batt_mV);
  esp_wifi_set_promiscuous(true);
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(motorAFI, OUTPUT);
  pinMode(motorABI, OUTPUT);
  pinMode(motorBFI, OUTPUT);
  pinMode(motorBBI, OUTPUT);

  // Optional EN pins
  if (ENA >= 0) { pinMode(ENA, OUTPUT); digitalWrite(ENA, HIGH); }
  if (ENB >= 0) { pinMode(ENB, OUTPUT); digitalWrite(ENB, HIGH); }

  // Quiet boot
  safeDriveOff();
  delay(50);

  // PWM setup (4 channels)
  ledcSetup(0, pwmFreq, pwmResolution); ledcAttachPin(motorAFI, 0);
  ledcSetup(1, pwmFreq, pwmResolution); ledcAttachPin(motorABI, 1);
  ledcSetup(2, pwmFreq, pwmResolution); ledcAttachPin(motorBFI, 2);
  ledcSetup(3, pwmFreq, pwmResolution); ledcAttachPin(motorBBI, 3);
  stopMotors();

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, TRANSMITTER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  wifi_promiscuous_filter_t filt{};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);
  esp_wifi_set_promiscuous(true);

  lastRxMs = millis();
  Serial.print("Boat STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Boat ready – waiting for joystick commands…");
}

void loop() {
  if ((millis() - lastRxMs) > COMM_TIMEOUT_MS) safeDriveOff();
}
