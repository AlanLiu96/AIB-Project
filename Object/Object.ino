#include <CircularBuffer.h> // ring buffer
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <FirebaseArduino.h> 
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h> // IMU for shake detection and magnetometer
LSM9DS1 imu;
#include <Adafruit_NeoPixel.h> // Neopixels for LED
#include <math.h> // used for breathing 
  
// Necessary Connections
// GPIO16 (D0) --> RST [deep sleep] 
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

// custom vars for magnetometer
float lastMagX = 0; //in Gauss
float lastMagY = 0;
float lastMagZ = 0;
float lastMagXChange = 0;
float lastMagYChange = 0;
float lastMagZChange = 0;
CircularBuffer<long,100> magReadX; // the derivative or change in last 100 reads (last second)
CircularBuffer<long,100> magReadY;
CircularBuffer<long,100> magReadZ;

// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

/** light configurations **/
#define NUMPIXELS (2)
#define PERIOD (2000)
const int led = BUILTIN_LED;
int brightness_init = 255;
int brightness = 0;    // how bright the LED is

/* Neopixel LEDs */
int led_pin = 12;
 // Parameter 3 = pixel type flags, add together as needed: // TODO: which one 
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, led_pin, NEO_GRBW + NEO_KHZ800);

// initial pattern
int pattern = 0; // enum, 0 --> start/neutral, 1 --> fade, 2 --> blink
int active = 1;

// Wifi setup 
String mac = WiFi.macAddress();
int status = WL_IDLE_STATUS;

// iteration variables
//Ticker looper; // used to call loop periodically
int loop_iter = 0;
int last_pattern = 0;
long last_active; // tracks the last time activity occurred.
long last_blink; // tracks the last time it blinked


long last_check;
long integrated_lag = 0;// TODO(alan): I don't understand how this is used

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

void setup() {
 Serial.begin(115200);
 delay(500);

 Serial.println();
 Serial.print("MAC: ");
 Serial.println(mac);
 Serial.println();

 pinMode(led, OUTPUT);

 /** Wifi Setup **/
 WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 Serial.print("connecting");
 while (WiFi.status() != WL_CONNECTED) {
 Serial.print(".");
 delay(500);
 }
 Serial.print("connected: ");
 Serial.println(WiFi.localIP());

 /** Firebase Setup **/
 Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

 // run below only if the mac is not already on firebase 
 Firebase.set("object/" + mac + "/pattern", 1); 
 Firebase.set("object/" + mac + "/brightness", brightness_init);
 // Firebase.set("object/" + mac + "/active", 1); // turn on initally

 /** IMU Setup **/
 imu.settings.device.commInterface = IMU_MODE_I2C;
 imu.settings.device.mAddress = LSM9DS1_M;
 imu.settings.device.agAddress = LSM9DS1_AG;
 if (!imu.begin()) 
 {
    Serial.println("Failed to communicate with LSM9DS1. Continuing anyway.");
    delay(100);
 }

 /* Neopixel LED Setup */
  strip.begin();
  strip.setBrightness(0);
  for (int i = 0; i < NUMPIXELS; i++) {
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    strip.setPixelColor(i, strip.Color(0, 0, 0, 255));
  }
  strip.show();// Initialize all pixels to 'off'

  last_check = last_active = last_blink = millis(); 
}

void loop() {
    long start = millis();
/** Update Vals **/ 
  if (millis() - last_check > 3000){
  /* Active Refresh */
//    active = Firebase.getInt("object/" + mac + "/active");
    // if user engages with exhibits, then timer is refreshed 
//    if (active == 1) { // comment out due to removal of deep sleep
//      Firebase.set("object/" + mac+ "/active", 0);
//      last_active = millis();
//    }

  /* Read Light Pattern */
    pattern = Firebase.getInt("object/" + mac + "/pattern");
    if (pattern == 0) {// only needed if manual control
      brightness = Firebase.getInt("object/" + mac + "/brightness");}

//    Serial.print(active); Serial.print('\t');
//    Serial.print(pattern); Serial.print('\t');
//    Serial.print(brightness); Serial.print('\t');
//    Serial.print(PERIOD); Serial.print('\t');
//    Serial.print('\n');

//    Firebase.set("object/" + mac + "/uptime", (millis() / 1000));
    last_check = millis();
    Firebase.set("object/" + mac + "/lag", (last_check - start));
    integrated_lag += last_check - start;
    

//    String location = "unknown";
  /* Wifi Signal Detection */
//    long rssi = getRSSI("FuturePod");
//    if (rssi > -45 && rssi != 0){
//      location = "FuturePod";
//      Serial.print("location: FuturePod!");
//    }
//
//    rssi = getRSSI("Mirror");
//    if (rssi > -40 && rssi != 0){
//      location = "Mirror";
//      Serial.print("location: Mirror!");
//    }

//    Firebase.setString("object/"+ mac + "/location", location);
//    Firebase.set("object/"+ mac + "/visitedLocations/"+ location, true);
  }

/** Set Light Pattern **/
 // set pattern from Firebase 
 if (pattern == 1){ // fade
    brightness = (exp(sin((millis() - integrated_lag) / ((float)PERIOD) * 2 * PI)) - 0.36787944) * 108.0; //http://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
 } 
 else if (pattern == 2){ // blink
    if (millis() - last_blink > 1000){
      brightness = brightness < 255 ? 255 : 0;
      last_blink = millis();
    }
 }
 // set the brightness of led:
 // analogWrite(led, brightness);
   for (int i = 0; i < NUMPIXELS; i++){
    strip.setPixelColor(i, strip.Color(0, 0,0, 255));
   }
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

/* Magnetometer for Electromagnet Detection*/
if ( imu.magAvailable() ){
 imu.readMag();

 // plan. check every next value to see if positive or negative shift
 // if it is, check current time and add to the buffer.
 // remove all times from buffer that are > 1 sec from current time. 
 int xFrequency = 0; //frequency
 int yFrequency = 0;
 int zFrequency = 0;

 if (imu.calcMag(imu.mx-2.5) * lastMagX < 0){
  long cur_time = millis();
  magReadX.push(cur_time);
  if (cur_time - magReadX.first() < 1000){ // 1000 ms  
    xFrequency = magReadX.size()/2;
    magReadX.clear();
  }
 }
 if (imu.calcMag(imu.my-2.5) * lastMagY < 0){
  long cur_time = millis();
  magReadY.push(cur_time);
  if (cur_time - magReadY.first() < 1000){ // 1000 ms  
    yFrequency = magReadY.size()/2;
    magReadY.clear();
  }
 }
 if (imu.calcMag(imu.mz-2.5) * lastMagZ < 0){
  long cur_time = millis();
  magReadZ.push(cur_time);
  while (cur_time - magReadZ.first() < 1000){ // 1000 ms  
    zFrequency = magReadZ.size()/2;
    magReadZ.clear();
  }
 }

 //TODO: calculate what to do with x,y,z Frequencies
 // Subtracted 2.5 since we can only output 0-5v on electromagnet
 lastMagX = imu.calcMag(imu.mx-2.5);
 lastMagY = imu.calcMag(imu.my-2.5);
 lastMagZ = imu.calcMag(imu.mz-2.5); 
}
 /* Shake Detection */ 
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
        active = 1; // Send to firebase/ kinect somehow?
        // TODO: Activate behavior on Shake? 
      }
      accelZDelta = 0;
    }
    lastPrint = millis(); // Update lastPrint time
 }

/** Loop Iteration Ctrls **/
//  Serial.println(brightness);
 // wait for 10 milliseconds to see the pattern effect
 delay(10);
}
