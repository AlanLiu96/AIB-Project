#include <ESP8266WiFi.h>
#include <SPI.h>
#include <Wire.h>

#define WIFI_SSID "yale wireless"
#define WIFI_PASSWORD ""

// Return RSSI or 0 if target SSID not found
int32_t getRSSI(String target_ssid) {
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++) {
    if (WiFi.SSID(network).equals( target_ssid)) {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}

void setup()
{
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(100);

  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
}

void loop () {
  // if you are connected, print out info about the connection:
   // print the received signal strength:
  long rssi = getRSSI("FuturePod");
  Serial.print("RSSI:");
  Serial.println(rssi);
  }
