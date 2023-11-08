/*
 * Example
 *
 * If you encounter any issues:
 * - check the readme.md at https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md
 * - ensure all dependent libraries are installed
 * - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#arduinoide
 * - see https://github.com/sinricpro/esp8266-esp32-sdk/blob/master/README.md#dependencies
 * - open serial monitor and check whats happening
 * - check full user documentation at https://sinricpro.github.io/esp8266-esp32-sdk
 * - visit https://github.com/sinricpro/esp8266-esp32-sdk/issues and check for existing issues or open a new one
 */

 // Custom devices requires SinricPro ESP8266/ESP32 SDK 2.9.6 or later

// Uncomment the following line to enable serial debug output
//#define ENABLE_DEBUG

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

#include <SinricPro.h>
#include "SinricProTemperaturesensor.h"
#include "WeatherMonitor.h"
#include <DHT.h>
#include <Keypad.h>


#define BAUD_RATE  115200

/* Sinric Pro definitions */
#define APP_KEY    "7a7caefc-db9f-4372-b86d-41393f1f74cd"
#define APP_SECRET "8fede556-9e2f-4613-b78b-aeebc5cd2dbb-84d4844f-28c7-4350-9a96-d69387f56bbb"
#define DEVICE_ID  "653846228332c2648adaa2a7"
WeatherMonitor &weatherMonitor = SinricPro[DEVICE_ID];  // make instance of SinricPro device

/* Wifi Credentials */
#define SSID       "Pixel_7137" //"PSU-IoT"
#define PASS       "tc9h7msz9rpug8x" //"9SFkew1Hi2HyRANA"

/* Pin definitions */
#define rainAnalog 21
#define rainDigital 17

/* DHT definitions */
#define EVENT_WAIT_TIME   60000               // send event every 60 seconds
#define DHT_PIN           4                   // located on pin 4
DHT dht;                                      // make instance of DHT sensor


/*************
 * Prototypes *
 *************/

bool check_interval(unsigned long* previousMillis, long interval);
void handleTemperaturesensor(void);
void handleRainSensor(void);
void handleKeypad(void);
void handleDisplay(void);



/*************
 * Variables *
 ***********************************************
 * Global variables to store the device states *
 ***********************************************/

// ToggleController
std::map<String, bool> globalToggleStates;

/* DHT device */
bool deviceIsOn;                              // Temeprature sensor on/off state
float temperature;                            // actual temperature
float humidity;                               // actual humidity
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

/* LM393 Rain Sensor */
unsigned long LM393_previous_millis = 0;        // will store last time rain sensor was checked
const long LM393_sample_interval = 20 * 1000;   // interval at which to sample (in milliseconds)

/* Keypad Matrix */
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {5, 18, 19, 16}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {14, 32, 15, 33}; //connect to the column pinouts of the keypad
//board#: 5, 18, 19, 16   //Schematic#: 9, 8, 7, 6
//board#: 14, 32, 15, 33  //Schematic#: 5, 4, 3, 2

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

/*************
 * Callbacks *
 *************/

// ToggleController
bool onToggleState(const String& deviceId, const String& instance, bool &state) {
  Serial.printf("[Device: %s]: State for \"%s\" set to %s\r\n", deviceId.c_str(), instance.c_str(), state ? "on" : "off");
  globalToggleStates[instance] = state;
  return true;
}

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too many events
 * - Get actual temperature and humidity and check if these values are valid
 * - Compares actual temperature and humidity to last known temperature and humidity
 * - Send event to SinricPro Server if temperature or humidity changed
 */
void handleTemperaturesensor() {
  if (deviceIsOn == false) return; // device is off...do nothing

  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //only check every EVENT_WAIT_TIME milliseconds

  temperature = dht.getTemperature();          // get actual temperature in °C
//  temperature = dht.getTemperature() * 1.8f + 32;  // get actual temperature in °F
  humidity = dht.getHumidity();                // get actual humidity

  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("DHT reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  if (temperature == lastTemperature && humidity == lastHumidity) 
  {
    //Serial.println("Same temp and humidity as last checkin");
  }

  bool success = weatherMonitor.sendTemperatureEvent(temperature, humidity); // send event
  if (success) {  // if event was sent successfuly, print temperature and humidity to serial
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // if sending event failed, print error message
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
    return;
  }

  lastTemperature = temperature;  // save actual temperature for next compare
  lastHumidity = humidity;        // save actual humidity for next compare
  lastEvent = actualMillis;       // save actual time for next compare
}

/* handleRainSensor()
 * - Checks if time since last call > LM393_sample_interval to prevent sending too many events
 * - Get actual rain sensor values (doesn't check if they are valid)
 * - Prints values to Serial
 * - Other functionality not yet implemented
 * - TODO: - Add input validation
 *         - Add rain sensitivity adjustment
 */
void handleRainSensor(){
  /* Determine if enough time has passed since last check-in */
  if (check_interval(&LM393_previous_millis, LM393_sample_interval)) {
    int rainAnalogVal = analogRead(rainAnalog);
    int rainDigitalVal = digitalRead(rainDigital);
    
    Serial.print("Analog: ");
    Serial.print(rainAnalogVal);
    Serial.print("\tDigitial: ");
    Serial.println(rainDigitalVal);
  }
}

/* handleKeypad()
 * - Checks if a key has been pressed
 * - Prints key to Serial
 * - Other functionality not yet implemented
 * - TODO: Add menu system with options to adjust settings:
 *          - Rain sensitivity
 *          - Temperature sensor upper and lower limits
 *          - Manually open/close window
 *          - Timeout/Sleep mode?
 */
void handleKeypad(){
  char customKey = customKeypad.getKey();
  
  if (customKey){
    Serial.println(customKey);
  }

  //TODO: Add menu system
}

/* handleDisplay()
 * - TODO: - Display current temperature and humidity on the display
 *         - Other functionality not yet implemented
 *  
 */
void handleDisplay(){
  //TODO: Add display functionality
}

/**********
 * Events *
 *************************************************
 * Examples how to update the server status when *
 * you physically interact with your device or a *
 * sensor reading changes.                       *
 *************************************************/

// TemperatureSensor
void updateTemperature(float temperature, float humidity) {
  weatherMonitor.sendTemperatureEvent(temperature, humidity);
}

// PushNotificationController
void sendPushNotification(String notification) {
  weatherMonitor.sendPushNotification(notification);
}

// ToggleController
void updateToggleState(String instance, bool state) {
  weatherMonitor.sendToggleStateEvent(instance, state);
}

/********* 
 * Setup *
 *********/

void setupSinricPro() {
  // ToggleController
  weatherMonitor.onToggleState("toggleInstance1", onToggleState);

  SinricPro.onConnected([]{ Serial.printf("[SinricPro]: Connected\r\n"); });
  SinricPro.onDisconnected([]{ Serial.printf("[SinricPro]: Disconnected\r\n"); });
  SinricPro.begin(APP_KEY, APP_SECRET);
};

void setupWiFi() {
  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
  #endif

  WiFi.begin(SSID, PASS);
  Serial.printf("[WiFi]: Connecting to %s", SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  Serial.printf("connected\r\n");
}

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(rainDigital,INPUT);
  pinMode(rainAnalog,INPUT);

  setupWiFi();
  setupSinricPro();
  sendPushNotification("ESP Device is online");

  // Initial readings
  handleRainSensor();   // check rain sensor
  handleTemperaturesensor(); // check temperature sensor
}

/* checks if enough time has passed since last occurance, returns true/false*/
bool check_interval(unsigned long* previousMillis, long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - *previousMillis >= interval) {
    *previousMillis = currentMillis;
    return true;
  }
  return false;
}

/********
 * Loop *
 ********/
void loop() {
  /* Perform Sinric Pro actions*/
  SinricPro.handle();

  /* Check for input from the keypad */
  handleKeypad();

  /* Check for input from the rain sensor */
  handleRainSensor();

  /* Measure temperature and humidity. */
  handleTemperaturesensor();

  /* Display temperature and humidity on the display */
  handleDisplay();

  }