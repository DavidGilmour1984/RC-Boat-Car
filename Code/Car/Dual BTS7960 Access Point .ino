#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// WIFI AP
// =====================================================
const char* ssid = "ESP32_RC_CAR";
WebServer server(80);

// =====================================================
// LEFT MOTOR
// =====================================================
#define L_RPWM 25
#define L_LPWM 26
#define L_REN  27
#define L_LEN  14

// =====================================================
// RIGHT MOTOR
// (UPDATED EN PINS: 4 + 23)
// =====================================================
#define R_RPWM 32
#define R_LPWM 33
#define R_REN  23
#define R_LEN  4

// =====================================================
// BATTERY MONITOR (VP PIN)
// =====================================================
#define VBAT_PIN 36

// =====================================================
// PWM CONFIG
// =====================================================
const int freq = 1000;
const int resolution = 8;

const int chL_R = 0;
const int chL_L = 1;
const int chR_R = 2;
const int chR_L = 3;

// =====================================================
// CONTROL INPUTS
// =====================================================
int throttle = 0;   // -255 to +255
int steer = 0;      // -255 to +255

// =====================================================
// MOTOR MIX FUNCTION
// =====================================================
void setMotor(int rpwm, int lpwm, int val) {

  val = constrain(val, -255, 255);

  if (val >= 0) {
    ledcWrite(lpwm, 0);
    ledcWrite(rpwm, val);
  } else {
    ledcWrite(rpwm, 0);
    ledcWrite(lpwm, -val);
  }
}

void stopAll() {
  ledcWrite(chL_R, 0);
  ledcWrite(chL_L, 0);
  ledcWrite(chR_R, 0);
  ledcWrite(chR_L, 0);
}

// =====================================================
// UPDATE MOTORS (DIFFERENTIAL STEERING)
// =====================================================
void updateMotors() {

  int left = throttle + steer;
  int right = throttle - steer;

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  setMotor(chL_R, chL_L, left);
  setMotor(chR_R, chR_L, right);
}

// =====================================================
// BATTERY VOLTAGE
// =====================================================
float readBattery() {

  int raw = analogRead(VBAT_PIN);
  float v = (raw / 4095.0) * 3.3;

  // adjust for your voltage divider
  return v * 4.2;
}

// =====================================================
// HTML PAGE
// =====================================================
String page() {

  float vbat = readBattery();

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
body{
font-family:Helvetica;
text-align:center;
background:#f5f7fb;
margin-top:30px;
}

.card{
background:white;
margin:15px;
padding:20px;
border-radius:15px;
box-shadow:0 4px 10px rgba(0,0,0,0.1);
}

input{
width:85%;
height:50px;
}

h1{font-size:40px;}

.value{
font-size:30px;
}

</style>
</head>

<body>

<h1>RC CAR</h1>

<div class="card">
<h2>Battery</h2>
<div class="value">VBAT V</div>
</div>

<div class="card">
<h2>Throttle</h2>
<input type="range" min="-255" max="255" value="THROTTLE"
oninput="fetch('/t?val='+this.value)">
</div>

<div class="card">
<h2>Steering</h2>
<input type="range" min="-255" max="255" value="STEER"
oninput="fetch('/s?val='+this.value)">
</div>

<div class="card">
<button onclick="fetch('/stop')"
style="width:200px;height:70px;font-size:25px;">
STOP
</button>
</div>

<script>
setInterval(()=>location.reload(),2000);
</script>

</body>
</html>
)rawliteral";

  html.replace("VBAT", String(vbat, 2));
  html.replace("THROTTLE", String(throttle));
  html.replace("STEER", String(steer));

  return html;
}

// =====================================================
// HANDLERS
// =====================================================
void handleRoot() {
  server.send(200, "text/html", page());
}

void setThrottle() {
  throttle = server.arg("val").toInt();
  updateMotors();
  server.send(200, "text/plain", "OK");
}

void setSteer() {
  steer = server.arg("val").toInt();
  updateMotors();
  server.send(200, "text/plain", "OK");
}

void stopCmd() {
  throttle = 0;
  steer = 0;
  stopAll();
  server.send(200, "text/plain", "OK");
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  ledcSetup(chL_R, freq, resolution);
  ledcSetup(chL_L, freq, resolution);
  ledcSetup(chR_R, freq, resolution);
  ledcSetup(chR_L, freq, resolution);

  ledcAttachPin(L_RPWM, chL_R);
  ledcAttachPin(L_LPWM, chL_L);
  ledcAttachPin(R_RPWM, chR_R);
  ledcAttachPin(R_LPWM, chR_L);

  pinMode(VBAT_PIN, INPUT);

  stopAll();

  WiFi.softAP(ssid);

  server.on("/", handleRoot);
  server.on("/t", setThrottle);
  server.on("/s", setSteer);
  server.on("/stop", stopCmd);

  server.begin();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  server.handleClient();
}
