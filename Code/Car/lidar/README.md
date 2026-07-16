# ESP32 GPS Autonomous Rover

A complete autonomous ground rover system built around two ESP32s, a Quectel L80-R GPS, LIS3MDL compass, dual BTS7960 motor drivers, and a D500 (STL-19P) LiDAR for hazard avoidance. Control and telemetry run over ESP-NOW. The operator interface is a single HTML file opened in Chrome.

---

## System Overview

```
Laptop (Chrome)
    └── rover_gui.html
          │  Web Serial API
          ▼
    Bridge ESP32  (USB/COM port)
          │  ESP-NOW 2.4GHz
          ▼
    Car ESP32  (on rover)
          ├── LIS3MDL compass      (I2C)
          ├── Quectel L80-R GPS    (UART2)
          ├── D500 LiDAR           (UART1) — hazard avoidance
          ├── BTS7960 Left motor driver
          └── BTS7960 Right motor driver
```

---

## Hardware

### Car ESP32
| Component | Connection |
|---|---|
| Left motor RPWM | GPIO 25 |
| Left motor LPWM | GPIO 26 |
| Left motor REN | GPIO 27 |
| Left motor LEN | GPIO 14 |
| Right motor RPWM | GPIO 32 |
| Right motor LPWM | GPIO 33 |
| Right motor REN | GPIO 23 |
| Right motor LEN | GPIO 4 |
| GPS TX → ESP32 RX2 | GPIO 16 |
| GPS RX → ESP32 TX2 | GPIO 17 |
| Compass SDA | GPIO 21 |
| Compass SCL | GPIO 22 |
| Battery monitor | GPIO 36 (VP) |
| LiDAR Tx → ESP32 RX1 | GPIO 18 |
| LiDAR P5V | 5V |
| LiDAR GND | GND |

**LiDAR PWM pin is not connected.** Left floating, the D500 uses internal speed control at a fixed 10Hz — plenty for hazard avoidance, and one less wire to run. Only wire the PWM pin if you later want closed-loop control of the scan rate.

### Battery Monitor Voltage Divider
The 3S LiPo (max 12.6V) must be divided down to ≤3.3V for GPIO 36.

```
Battery+ ──┤ 10kΩ ├──┬── GPIO 36 (VP)
                      │
                    1kΩ
                      │
                     GND
```

Ratio = 1/(10+1) = **0.0909**. At 12.6V → 1.15V (safe).
Adjust `VDIV_RATIO` in `car_esp32.ino` if you use different resistors.

| Threshold | Action |
|---|---|
| < 10.5V | Warning in GUI, speed limited to 50% |
| < 9.9V | Hard stop, motors idle |

### Bridge ESP32
Standard 30-pin ESP32 DevKit connected via USB to laptop. No additional wiring needed.

---

## Software Setup

### Arduino Libraries (Car ESP32)
Install via Arduino Library Manager:
- **TinyGPSPlus** by Mikal Hart
- **Adafruit LIS3MDL**
- **Adafruit Unified Sensor**

### Board
- ESP32 Dev Module (both boards)
- Upload speed: 921600
- Flash size: 4MB

---

## First-Time Setup

### Step 1 — Get MAC addresses

Flash `print_mac.ino` to each ESP32 in turn and open Serial Monitor at 115200 baud. It prints the board's MAC address every 3 seconds:

```
MAC Address: AA:BB:CC:DD:EE:FF
```

Copy the **car's** MAC into `bridge_esp32.ino`:
```cpp
uint8_t roverMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
```

Copy the **bridge's** MAC into `car_esp32.ino`:
```cpp
uint8_t bridgeMAC[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
```

Reflash both boards with `car_esp32.ino` / `bridge_esp32.ino` containing the correct MACs.

### Step 2 — Compass Calibration

Flash the standalone `compass_calibration.ino` sketch first:

- SSID: `ROVER_DIAG` (no password)
- Open `http://192.168.4.1` in a browser
- Click **Start Calibration**, rotate the rover slowly through a full 360° (keep it level)
- Click **Stop & Save**
- Calibration offsets are stored in EEPROM and survive power cycles, including a later reflash of `car_esp32.ino`

**Recalibrate any time** by reflashing `compass_calibration.ino`, repeating the steps, then reflashing `car_esp32.ino`.

### Step 3 — Launch the GUI

1. Open `rover_gui.html` in **Chrome** or **Edge** (Web Serial API required)
2. Plug the Bridge ESP32 into your laptop via USB
3. Click **CONNECT** and select the COM port
4. The status bar shows **COMMS ONLINE** when the bridge is ready

---

## Driving

### Manual Mode
- **Arrow keys** — drive the rover
  - ↑ / ↓ — both motors forward / reverse
  - ← / → alone — spin left / spin right in place
  - ↑ + ← or ↑ + → — forward with left or right bias
- **Escape** — Emergency Stop
- **Speed slider** — sets drive speed (0–255 PWM)
- **Heading lock** — enable to hold a compass bearing while driving straight; set the target bearing with the slider

### Auto Mode
1. Switch to **Auto** mode
2. Open the **Waypoints drawer** (click the handle at the bottom of the map)
3. Click **+ Place** then click points on the map to set the route
4. Each waypoint shows its number, coordinates, speed and arrival radius
5. Double-click any waypoint flag or click ✎ to edit its individual speed, radius and label
6. Click **⤢ Move** to drag waypoints to new positions
7. Click **START MISSION** — button arms (turns amber), click again within 3 seconds to confirm
8. Rover drives to each waypoint in sequence

### Auto-Advance vs Manual Advance
| Setting | Behaviour |
|---|---|
| Auto-advance ON | Rover arrives, immediately proceeds to next waypoint |
| Auto-advance OFF | Rover arrives and stops; **▶ Advance** button appears to proceed manually |

The **⏸ Pause** button halts the rover mid-leg at any time. **▶ Resume** to continue.

### Loop Mode
Enable in Settings. After reaching the last waypoint the rover returns to waypoint 1 and repeats the route indefinitely. The map shows the closing leg with a direction arrow. Mission runs until you click **■ Stop Mission** or hit Escape.

---

## Hazard Avoidance (LiDAR)

The D500 LiDAR scans continuously and reports nearby obstacles as clustered points, both to the drive logic (for automatic overrides) and to the GUI (for visual awareness).

### Enabling
In **Settings**:
- **Hazard avoidance (LiDAR)** — toggle on/off. **Defaults to OFF** — bench-test against real hardware before relying on it, same as stuck detection and battery alerts.
- **Hazard field of view** — sets how wide a slice of the scan (centred on straight ahead) is processed and shown, from ±5° to ±90°.

### How it overrides driving
A narrow **±15° cone dead ahead** (independent of the wider FOV slider) is checked every loop:

| Distance dead ahead | Effect |
|---|---|
| > 900mm | No effect — full speed as commanded |
| 400–900mm | Forward speed scaled down proportionally |
| < 400mm | Forward motion fully blocked |

Steering and reverse are **never** touched — the rover can still turn or back away even while blocked from driving forward into something. These thresholds are set in `car_esp32.ino`:
```cpp
#define HAZARD_STOP_MM 400
#define HAZARD_SLOW_MM 900
#define HAZARD_CENTER_HALF_DEG 15
```

### GUI display
A small radar-style widget next to the compass shows live obstacle clusters within the configured FOV — green/amber/red by distance — plus the closest object's range. Up to 16 clusters are reported per scan.

### Mounting offset
If the field looks rotated relative to the rover's actual heading, adjust in `car_esp32.ino`:
```cpp
#define LIDAR_MOUNT_OFFSET_DEG 270
```

---

## Route Planning

The map displays:
- **Amber dotted line** — planned route between waypoints in order
- **Direction arrows** — at the midpoint of each leg showing travel direction
- **Green dot** — live rover position
- **Green dashed line** — rover's current heading toward target waypoint
- **Numbered markers** — each waypoint (green = current target, blue = upcoming)

Map defaults to **St Peter's School, Cambridge, New Zealand**. The rover marker appears as soon as a valid GPS fix is received.

---

## Missions (Save / Load)

Type a mission name in the text box, then:

- **💾 Save** — downloads a `.csv` file containing all waypoints and settings
- **📂 Load** — opens a file picker; loading rebuilds all waypoints on the map and restores all settings

### Mission CSV format
```
mission_name,My Route
global_speed,150
global_radius,7
auto_advance,true
loop_mode,false
heading_lock,false
lock_bearing,0

wp,lat,lon,speed,radius,label
1,-37.8908,175.4326,150,7,Start
2,-37.8915,175.4331,120,8,Gate
3,-37.8921,175.4318,150,7,
```

---

## Telemetry

The right panel shows live data from the rover:

| Field | Description |
|---|---|
| Heading | Compass bearing (compass rose + degrees) |
| Distance | Metres to current waypoint |
| Bearing | Compass bearing to current waypoint |
| Speed | GPS speed in m/s (zeroed below 0.4 m/s when stationary) |
| Sats / HDOP | GPS satellite count and accuracy |
| Battery | Voltage with colour bar (green → amber → red) |
| Signal | ESP-NOW RSSI with bar graph |
| Position | Live lat/lon |
| Hazards | Nearby obstacle clusters (angle + distance) within the configured FOV |

### Trip Log
Click **⬇ Trip Log** in the header to download a CSV of position, speed, heading and battery logged every 5 seconds during the mission.

---

## Safety

| Event | Response |
|---|---|
| Comms lost > 5s | Motors idle (PWM=0, enables stay HIGH) |
| Battery < 10.5V | Speed limited to 50%, GUI warning |
| Battery < 9.9V | Hard stop |
| Obstacle < 400mm dead ahead | Forward motion blocked (steering/reverse unaffected) |
| Escape key | Emergency stop sent immediately |
| ⚡ E-STOP button | Emergency stop, mission aborted |

**Comms loss behaviour:** motors go to idle (not a hard brake) to avoid mechanical shock. The rover will restart driving as soon as comms are restored and a command is received.

---

## Tuning

### Arrival Radius
Default 7m. Increase on open ground; the system also automatically scales the radius up when HDOP is poor (GPS accuracy degraded). Adjustable globally via slider or per-waypoint via the edit popup.

### PID Steering (Auto Mode)
In `car_esp32.ino`:
```cpp
float pid_Kp = 1.8f;   // proportional — increase for sharper corrections
float pid_Ki = 0.0f;   // integral — leave at 0 unless rover consistently misses targets
float pid_Kd = 0.4f;   // derivative — increase to reduce oscillation/weaving
```

### Speed Threshold (GPS noise filter)
```cpp
speed_mps = (raw < 0.4f) ? 0.0f : raw;
```
Increase `0.4f` if you see phantom speed readings when stationary.

### Hazard Avoidance Thresholds
See the [Hazard Avoidance](#hazard-avoidance-lidar) section above for `HAZARD_STOP_MM`, `HAZARD_SLOW_MM`, `HAZARD_CENTER_HALF_DEG`, and `LIDAR_MOUNT_OFFSET_DEG`.

### Hazard Cluster Capacity
Up to 16 obstacle clusters are sent per telemetry packet (`HAZARD_MAX_CLUSTERS` in both `car_esp32.ino` and `bridge_esp32.ino` — must match). The telemetry packet is currently ~88 bytes against ESP-NOW's 250-byte cap, so there's headroom to increase this further if needed.

---

## File Summary

| File | Purpose |
|---|---|
| `car_esp32.ino` | Rover firmware — navigation, motors, GPS, compass, LiDAR hazard avoidance, ESP-NOW |
| `bridge_esp32.ino` | Bridge firmware — Serial ↔ ESP-NOW passthrough |
| `rover_gui.html` | Operator GUI — open in Chrome, no server needed |
| `compass_calibration.ino` | Standalone sketch for compass calibration (flash before `car_esp32.ino`, or any time recalibration is needed) |
| `print_mac.ino` | Standalone sketch — flash to either board to read its WiFi MAC address for pairing |
| `d500_lidar_reader.html` | Standalone bench-test tool — visualizes raw LiDAR output over direct USB/COM, independent of the rover system. Useful for verifying the LiDAR works before wiring it into the rover. |
