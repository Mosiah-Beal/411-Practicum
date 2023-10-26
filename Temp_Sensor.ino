/*
 * Example for how to use SinricPro Temperaturesensor device:
 * - setup a temperature sensor device
 * - send temperature event to SinricPro server when temperature has changed
 * 
 *
 * DHT Library used in this example: https://github.com/markruys/arduino-DHT
 * 
 * If you encounter any issues:
 * - check the readme.md at https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md
 * - ensure all dependent libraries are installed
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#arduinoide
 *   - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#dependencies
 * - open serial monitor and check whats happening
 * - check full user documentation at https://sinricpro.github.io/esp8266-esp32-sdk
 * - visit https://github.com/sinricpro/esp8266-esp32-sdk/issues and check for existing issues or open a new one
 */



#ifdef ENABLE_DEBUG
 #define DEBUG_ESP_PORT Serial
 #define NODEBUG_WEBSOCKETS
 #define NDEBUG
#endif

#include <Arduino.h>
#ifdef ESP8266 
 #include <ESP8266WiFi.h>
#endif 
#ifdef ESP32   
 #include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProTemperaturesensor.h"
#include "DHT.h" // https://github.com/markruys/arduino-DHT

#define WIFI_SSID         "Pixel_7137" //"PSU-IoT"
#define WIFI_PASS         "tc9h7msz9rpug8x" //"9SFkew1Hi2HyRANA"
#define APP_KEY           "7a7caefc-db9f-4372-b86d-41393f1f74cd"
#define APP_SECRET        "8fede556-9e2f-4613-b78b-aeebc5cd2dbb-84d4844f-28c7-4350-9a96-d69387f56bbb"
#define TEMP_SENSOR_ID    "65386353b1138c00c95e2e88" 
#define BAUD_RATE         115200              // Change baudrate to your need (used for serial monitor)
#define EVENT_WAIT_TIME   60000               // send event every 60 seconds

#define DHT_PIN           4

DHT dht;                                      // DHT sensor

bool deviceIsOn;                              // Temeprature sensor on/off state
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

/* bool onPowerState(String deviceId, bool &state) 
 *
 * Callback for setPowerState request
 * parameters
 *  String deviceId (r)
 *    contains deviceId (useful if this callback used by multiple devices)
 *  bool &state (r/w)
 *    contains the requested state (true:on / false:off)
 *    must return the new state
 * 
 * return
 *  true if request should be marked as handled correctly / false if not
 */
 
bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state?"on":"off");
  deviceIsOn = state; // turn on / off temperature sensor
  return true; // request handled properly
}

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too much events
 * - Get actual temperature and humidity and check if these values are valid
 * - Compares actual temperature and humidity to last known temperature and humidity
 * - Send event to SinricPro Server if temperature or humidity changed
 */
void handleTemperaturesensor() {
  if (deviceIsOn == false) return; // device is off...do nothing

  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  Serial.println("Timer expired");
  temperature = dht.getTemperature();          // get actual temperature in °C
//  temperature = dht.getTemperature() * 1.8f + 32;  // get actual temperature in °F
  humidity = dht.getHumidity();                // get actual humidity

  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("DHT reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature && humidity == lastHumidity) 
  {
    Serial.println("Same temp and humidity as last checkin");
    lastTemperature = temperature;  // save actual temperature for next compare
    lastHumidity = humidity;        // save actual humidity for next compare
    lastEvent = actualMillis;       // save actual time for next compare  
    return; // if no values changed do nothing...
  }

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // get temperaturesensor device
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // send event
  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}


// setup function for WiFi connection
void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  mySensor.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
     
  SinricPro.begin(APP_KEY, APP_SECRET);  
}

// main setup function
void setup() {
  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n");
  dht.setup(DHT_PIN);

  setupWiFi();
  setupSinricPro();
  Serial.println("Temp_Sensor Sketch");
  Serial.print("Device is "); //Serial.println(onPowerState);
  
}

void loop() {
  SinricPro.handle();
  handleTemperaturesensor();
}
