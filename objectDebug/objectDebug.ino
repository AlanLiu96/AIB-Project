#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include <FirebaseArduino.h>

#include <Adafruit_NeoPixel.h> // Neopixels for LED

#include <Ticker.h>

#include <math.h>

// Set these to run example.
#define FIREBASE_HOST "aib-object.firebaseio.com"
#define FIREBASE_AUTH "Jh04XGLCRZco3K0EBwxXrSoaEY8zGUi9x5MS1WcJ"
#define WIFI_SSID "yale wireless"
#define WIFI_PASSWORD ""

String mac = WiFi.macAddress();
int status = WL_IDLE_STATUS;

/* Neopixel LEDs */
#define PIN            12
#define NUMPIXELS      1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);

Ticker looper; // Ticker calls a function periodically

int pattern = 0;
int brightness = 255;
int period = 2000;
int active = 0;


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(20);
  }

  Serial.println();
  Serial.print("MAC: ");
  Serial.println(mac);
  Serial.println();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  /** Firebase Setup **/
  Serial.print("firebase registering ");
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Serial.println("registered");

  Firebase.set("object/" + mac + "/pattern", pattern);
  Firebase.set("object/" + mac + "/brightness", brightness);
  Firebase.set("object/" + mac + "/active", active);
  Firebase.set("object/" + mac + "/period", period);

  /* Neopixel LED Setup */
  strip.begin();
  strip.setBrightness(0);
  strip.show();

}

long lastcheck;
long integrated_lag;

void loop() { // empty for esp8266 deep sleep? 
}

void tickerLoop() {
  long start = millis();
  if (millis() - lastcheck > 10000) {
    active = Firebase.getInt("object/" + mac + "/active");

    pattern = Firebase.getInt("object/" + mac + "/pattern");
    brightness = Firebase.getInt("object/" + mac + "/brightness");
    period = Firebase.getInt("object/" + mac + "/period");
    Serial.print(active); Serial.print('\t');
    Serial.print(pattern); Serial.print('\t');
    Serial.print(brightness); Serial.print('\t');
    Serial.print(period); Serial.print('\t');
    Serial.print('\n');
    Firebase.set("object/" + mac + "/uptime", (millis() / 1000));
    Firebase.set("object/" + mac + "/lag", (millis() - start));
    integrated_lag += millis() - start;
    lastcheck = millis();
  }

  if (active == 0) {
    delay(1000);
    return;
  }

  float val = 0;
  if (pattern == 1) {
    val = (exp(sin((millis() - integrated_lag) / ((float)period) * 2 * PI)) - 0.36787944) * 108.0 * brightness / 255; //http://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
  } else {
    val = ((millis() - integrated_lag) % period < (period / 2)) ? brightness : 0;
  }
  strip.setBrightness(val);
  for (int i = 0; i < NUMPIXELS; i++) {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(i, strip.Color(0, 0, 0, 255));
  }
  strip.show(); // This sends the updated pixel color to the hardware.
}