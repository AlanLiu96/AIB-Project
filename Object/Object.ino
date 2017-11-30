#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <FirebaseArduino.h> 
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h> // IMU for shake detection and magnetometer
LSM9DS1 imu;
#include <Adafruit_NeoPixel.h> // Neopixels for LED

// Necessary Connections
// GPIO13 (D0) --> RST [deep sleep] 
// LED_PIN --> 

// Set these to run example.
#define FIREBASE_HOST "aib-object.firebaseio.com"
#define FIREBASE_AUTH "Jh04XGLCRZco3K0EBwxXrSoaEY8zGUi9x5MS1WcJ"
#define WIFI_SSID "yale wireless"
#define WIFI_PASSWORD ""

// IMU Setup
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED

// custom variables to get a good read on shakes
#define PRINT_SPEED 100 // 250 ms between prints
#define THRESHOLD (750.0/PRINT_SPEED)
#define TIMES_PER_SEC (3)
static unsigned long lastPrint = 0; // Keep track of print time
float lastShake = 0;
float lastZ = 1;
float accelZDelta = 0;

// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

/** light configurations **/
const int led = BUILTIN_LED;
int brightness_init = 255;
int brightness = 0;    // how bright the LED is
int fadeAmount = 100;    // how many points to fade the LED by

/* Neopixel LEDs */
 led_pin = 6;
 // Parameter 3 = pixel type flags, add together as needed: // TODO: which one 
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
 Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, led_pin, NEO_GRB + NEO_KHZ800);

// initial pattern
int pattern = 0; // enum, 0 --> start/neutral, 1 --> fade, 2 --> blink

// Wifi setup 
String mac = WiFi.macAddress();
int status = WL_IDLE_STATUS;

// iteration variables
int loop_iter = 0;
int last_pattern = 0;

void setup() {
 Serial.begin(115200);
 delay(500);

 // wait for Serial to reconnect after deep sleep
 while (!Serial) { 
    delay(20);
 } 

 Serial.println();
 Serial.print("MAC: ");
 Serial.println(mac);
 Serial.println();

 pinMode(led, OUTPUT);

 /** Wifi Setup **/
 WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 Serial.print("connecting");
 int sleepctr = 0;
 while (WiFi.status() != WL_CONNECTED) {
 Serial.print(".");
 delay(500);
 sleepctr ++;
   if (sleepctr >= 20){ // 10 seconds to connect
    ESP.deepSleep(30e6); // Deepsleeps for 30 seconds if it can't connect
   }
 }
 Serial.print("connected: ");
 Serial.println(WiFi.localIP());

 /** Firebase Setup **/
 Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

 // run below only if the mac is not already on firebase 
// Firebase.set(mac + "/pattern", 0); 
// Firebase.set(mac + "/brightness", brightness_init);
// Firebase.set(mac + "/active", 1); // turn on initally

 /** IMU Setup **/
 imu.settings.device.commInterface = IMU_MODE_I2C;
 imu.settings.device.mAddress = LSM9DS1_M;
 imu.settings.device.agAddress = LSM9DS1_AG;
// while (!imu.begin()) // commented out for non-imu demo
// {
//    Serial.println("Failed to communicate with LSM9DS1.");
//    delay(100);
// }
 /* Neopixel LED Setup */
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
 strip.setPixelColor(0, 0, 0, 0, 255); // white to 255 for led 0 
 strip.setPixelColor(1, 0, 0, 0, 255); // white to 255 for led 1
}

void loop() {
 /** Deep Sleep Timeout **/
 // TODO: confirm if we need a deepsleep here based on power consumption, if we do how much time & when? 
   int active = Firebase.getInt(mac + "/active");
 if (loop_iter > 600 || (loop_iter == 0 && !active) ){ // 60 seconds of inactivity or just woke up and nothing happened
    ESP.deepSleep(30e6); // sleep for 30 seconds in deep sleep
}

 /** Set Light Pattern **/
 // Read and set pattern from Firebase
 pattern = Firebase.getInt(mac+ "/pattern");
 if (pattern == 0){ // manual control
    brightness = Firebase.getInt(mac + "/brightness");
 } 
 else if (pattern == 1){ // fade
    brightness = brightness + fadeAmount;
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 1023) {
        fadeAmount = -fadeAmount;
   }
 } 
 else if (pattern == 2){ // blink
     brightness = brightness < 500 ? 500 : 0;
 }
 // set the brightness of led:
 analogWrite(led, brightness);
 strip.setBrightness(brightness);
 strip.show();


 /** Accelerometer for Pattern Detection **/
 if ( imu.accelAvailable() )
 {
    imu.readAccel();
 }
 else {
    Serial.println("Error - IMU not available");
 }

 if ((lastPrint + PRINT_SPEED) < millis())
 {
    lastShake ++;
    float curAccel = imu.calcAccel(imu.az);
    accelZDelta += abs(curAccel - lastZ);
    lastZ = curAccel;
    
    if (lastShake >= 1000/PRINT_SPEED/TIMES_PER_SEC){ //per second
      lastShake = 0; 
      if (accelZDelta > THRESHOLD/TIMES_PER_SEC){
        Serial.println("Shake Activated");
        active = 1;
        // TODO: Activate behavior on Shake? 
      }
      accelZDelta = 0;
    }
    lastPrint = millis(); // Update lastPrint time
 }

 /** Active Timer Refresh **/
 // if user engages with exhibits, then timer is refreshed 
 if (active == 1) {
    Firebase.set(mac+ "/active", 0);
    loop_iter = 0;
 }

 /** Loop Iteration Ctrls **/
 Serial.println(brightness);
 // wait for 100 milliseconds to see the pattern effect
 delay(100);
 loop_iter ++;
}
