// ============================================================
//  COMPASS CALIBRATION — LIS3MDL
//  Flash this to calibrate the compass.
//  Offsets are saved to EEPROM and will be loaded
//  automatically by the main rover firmware.
//
//  Steps:
//  1. Flash this sketch
//  2. Connect to WiFi: ROVER_DIAG (no password)
//  3. Open http://192.168.4.1 in a browser
//  4. Click Start Calibration
//  5. Slowly rotate the rover through a full 360°
//  6. Click Stop & Save
//  7. Flash car_esp32.ino to restore rover firmware
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <math.h>

#define LIS3MDL_ADDR  0x1C
#define EEPROM_SIZE   24
#define CAL_ADDR      0

const char* ssid = "ROVER_DIAG";

Adafruit_LIS3MDL lis3mdl;
WebServer server(80);

float offset_x = 0, offset_y = 0;
float min_x, max_x, min_y, max_y;
bool  calibrating = false;
float currentBearing = 0;

// ── EEPROM ───────────────────────────────────────────────────
void saveCalibration() {
  EEPROM.put(CAL_ADDR,     offset_x);
  EEPROM.put(CAL_ADDR + 4, offset_y);
  EEPROM.commit();
  Serial.print("Saved — offset_x: "); Serial.print(offset_x, 3);
  Serial.print("  offset_y: "); Serial.println(offset_y, 3);
}

void loadCalibration() {
  EEPROM.get(CAL_ADDR,     offset_x);
  EEPROM.get(CAL_ADDR + 4, offset_y);
  if (isnan(offset_x) || isnan(offset_y)) {
    offset_x = 0; offset_y = 0;
    Serial.println("No calibration in EEPROM — using defaults");
  } else {
    Serial.print("Loaded — offset_x: "); Serial.print(offset_x, 3);
    Serial.print("  offset_y: "); Serial.println(offset_y, 3);
  }
}

// ── WEB PAGE ─────────────────────────────────────────────────
String buildPage() {
  String h = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>Compass Calibration</title>";
  h += "<style>";
  h += "body{font-family:Arial,sans-serif;background:#f0f2f5;display:flex;justify-content:center;align-items:center;min-height:100vh;margin:0;}";
  h += ".card{background:#fff;border-radius:12px;padding:32px;box-shadow:0 4px 20px rgba(0,0,0,0.1);text-align:center;max-width:420px;width:90%;}";
  h += "h1{color:#0057cc;margin-top:0;font-size:22px;}";
  h += ".bearing{font-size:64px;font-weight:bold;color:#0057cc;margin:10px 0;}";
  h += ".offsets{font-size:13px;color:#666;margin:8px 0;}";
  h += ".status{font-size:15px;font-weight:bold;margin:16px 0;padding:10px;border-radius:6px;}";
  h += ".status.ready{background:#e6f9ee;color:#008a3c;}";
  h += ".status.cal{background:#fff3e0;color:#b85c00;}";
  h += "svg{margin:16px auto;display:block;}";
  h += "a{display:inline-block;margin:8px 6px;padding:14px 28px;border-radius:8px;font-size:15px;font-weight:bold;text-decoration:none;color:#fff;}";
  h += ".start{background:#0057cc;} .start:hover{background:#003d99;}";
  h += ".stop{background:#cc0000;} .stop:hover{background:#990000;}";
  h += "</style></head><body><div class='card'>";
  h += "<h1>🧭 Compass Calibration</h1>";

  // Compass rose SVG
  h += "<svg viewBox='0 0 160 160' width='160' height='160' xmlns='http://www.w3.org/2000/svg'>";
  h += "<circle cx='80' cy='80' r='75' fill='#f0f2f5' stroke='#b0bcd0' stroke-width='2'/>";
  h += "<text x='80' y='18' text-anchor='middle' font-size='14' font-weight='bold' fill='#cc0000' font-family='Arial'>N</text>";
  h += "<text x='80' y='152' text-anchor='middle' font-size='14' fill='#666' font-family='Arial'>S</text>";
  h += "<text x='148' y='85' text-anchor='middle' font-size='14' fill='#666' font-family='Arial'>E</text>";
  h += "<text x='12' y='85' text-anchor='middle' font-size='14' fill='#666' font-family='Arial'>W</text>";
  h += "<g id='needle' transform='rotate(0,80,80)'>";
  h += "<polygon points='80,18 75,80 80,95 85,80' fill='#cc0000'/>";
  h += "<polygon points='80,142 75,80 80,95 85,80' fill='#b0bcd0'/>";
  h += "</g>";
  h += "<circle cx='80' cy='80' r='5' fill='#fff' stroke='#666' stroke-width='2'/>";
  h += "</svg>";

  h += "<div class='bearing' id='bearing'>---</div>";
  h += "<div class='offsets'>Offsets: X=" + String(offset_x,3) + " Y=" + String(offset_y,3) + "</div>";

  if (calibrating) {
    h += "<div class='status cal'>⟳ CALIBRATING — rotate slowly 360°</div>";
    h += "<a class='stop' href='/stop'>■ Stop &amp; Save</a>";
  } else {
    h += "<div class='status ready'>✔ Ready — offsets loaded from EEPROM</div>";
    h += "<a class='start' href='/start'>▶ Start Calibration</a>";
  }

  h += "<script>";
  h += "function update(){";
  h += "fetch('/data').then(r=>r.json()).then(d=>{";
  h += "document.getElementById('bearing').textContent=d.bearing.toFixed(1)+'°';";
  h += "document.getElementById('needle').setAttribute('transform','rotate('+d.bearing+',80,80)');";
  h += "});";
  h += "}";
  h += "setInterval(update,250);update();";
  h += "</script>";
  h += "</div></body></html>";
  return h;
}

void handleRoot()  { server.send(200,"text/html",buildPage()); }
void handleData()  { server.send(200,"application/json","{\"bearing\":"+String(currentBearing,1)+"}"); }
void handleStart() {
  calibrating=true; min_x=min_y=1e9; max_x=max_y=-1e9;
  Serial.println("Calibration started — rotate 360°");
  server.sendHeader("Location","/"); server.send(303);
}
void handleStop()  {
  if (calibrating) {
    calibrating=false;
    offset_x=(max_x+min_x)/2;
    offset_y=(max_y+min_y)/2;
    saveCalibration();
  }
  server.sendHeader("Location","/"); server.send(303);
}

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(21, 22);

  if (!lis3mdl.begin_I2C(LIS3MDL_ADDR)) {
    Serial.println("LIS3MDL not found! Check wiring.");
    while(1) delay(1000);
  }
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  loadCalibration();

  WiFi.softAP(ssid);
  Serial.print("Connect to WiFi: "); Serial.println(ssid);
  Serial.print("Then open: http://"); Serial.println(WiFi.softAPIP());

  server.on("/",      handleRoot);
  server.on("/data",  handleData);
  server.on("/start", handleStart);
  server.on("/stop",  handleStop);
  server.begin();
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  sensors_event_t ev;
  lis3mdl.getEvent(&ev);

  float x = ev.magnetic.x;
  float y = ev.magnetic.y;

  if (calibrating) {
    if (x < min_x) min_x = x;
    if (x > max_x) max_x = x;
    if (y < min_y) min_y = y;
    if (y > max_y) max_y = y;
  }

  float cx = x - offset_x;
  float cy = y - offset_y;
  currentBearing = atan2(cy, cx) * 180.0f / PI;
  if (currentBearing < 0) currentBearing += 360.0f;

  delay(50);
}
