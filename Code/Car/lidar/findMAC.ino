// ============================================================
//  PRINT MAC — flash to either ESP32 to read its WiFi MAC
//  Open Serial Monitor at 115200 baud after flashing.
//  Use the printed address in the OTHER board's *MAC[] array.
// ============================================================

#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // Reprint every 3s in case you missed it / opened the monitor late
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  delay(3000);
}
