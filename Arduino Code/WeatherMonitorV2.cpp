// Info: Practicum Project for ECE 411
/**
 * @file WeatherMonitorV2.cpp
 * @author Mosiah Beal (mosiah@pdx.edu)
 * @version 0.5
 * @date 2023-11-30
 *
 * @brief Practicum Project for ECE 411
 *        Window Weather Monitor: Monitors the weather outside a window and control the window based on the weather.
 *        It utilizes a ESP32 Huzzah Feather board, a DHTxx temperature and humidity sensor, a LM393 rain sensor, a stepper motor,
 *        and a 4x4 keypad. It also has a 128x64 OLED display for debugging and displaying the current temperature and humidity.
 *        It uses two breakout boards, one for the stepper motor and one for the keypad. The OLED display and the keypad are connected over I2C. 
 * 
 *        The program stores the current state of the monitor in a struct called MonitorState. It also stores the user settings
 *        in a struct called userSettings. The program updates the monitor state based on the user settings and the sensor readings.
 *        It also sends the sensor readings to the Sinric Pro server.
 *        
 *        The program uses a menu system to allow the user to change the settings and test the components.
 *        The menu system is implemented using a tree of structs called MenuStructs. 
 *        Each menu has a title, a list of choices, a current selection, a pointer to the parent menu, a list of pointers to the child menus, 
 *        and a list of pointers to functions. The functions are called when the user selects the option associated with the function. 
 *        
 *        The menu system is implemented using a pointer to the current menu. The current menu pointer is updated when the user selects an option
 *        associated with a submenu. The current menu pointer will be updated to the parent menu when the user selects the exit option.
 * 
 *        The OLED display is used to display the current menu and the current selection. It is also used to display the current temperature and humidity.
 *        
 *        The keypad is used to navigate the menu system and select options. It is also used to set the user settings and test the components.
 * 
 */

/**
 * Window Weather Monitor
 * 
 * This program is designed to monitor the weather outside a window and control the window based on the weather.
 * It utilizes a ESP32 Huzzah Feather board, a DHTxx temperature and humidity sensor, a LM393 rain sensor, a stepper motor,
 * and a 4x4 keypad. It also has a 128x64 OLED display for debugging and displaying the current temperature and humidity.
 * 
 * Some components are utilized as proof of concept and would be replaced or altered for a final product.
 * (For example, the stepper motor is not strong enough to open the window, and the components would need to be housed outside the window)
 * 
 * It polls the DHT device for temperature and humidity readings and sends them to the Sinric Pro server.
 * It also sends the readings to the OLED display over I2C on address 0x3D.
 * The DHT polling is done every 60 seconds, and data is sent serially to the ESP32 on pin 4.
 * 
 * The rain sensor is a secondary system which can detect if it is raining. It utilizes a resistive panel and a
 * differential amplifier to detect the presence of water. Since it amplifies the difference between the two inputs, a 1 indicates
 * that the sensor is dry and a 0 indicates that the sensor is wet. The sensor is connected to the ESP32 on pin 17. 
 *
 * The stepper motor is a proof of concept for controlling the window, the final product would need a stronger motor and 
 * a dedicated power supply. There is breakout board for the stepper motor which is connected to the ESP32 on pins 2, 4, 15, and 16. 
 * The breakout board used the values sent to the pins to determine the direction and speed of the motor. Power is supplied to the
 * motor separately from the ESP32.
 * 
 * The keypad is connected to a breakout board which is connected to the ESP32 over I2C on address 0x34. The keypad is used to
 * control the monitor and adjust settings. It can also be used to manually open and close the window.
 * 
 * There are a couple of status LEDs on the PCB. The red LED indicates that the ESP32 is powered on. The green LED indicates that
 * window is open. The RGB LED is used to indicate the temperature status. Blue indicates that the temperature is below the lower limit,
 * Green is within the limits, and Red is above the upper limit. 
 * 
 * Other comments:
 * If the DHT detects unfavorable conditions (temperature too high or too low, humidity too high or too low) it will send a push notification
 * and close the window. Similarly, if the rain sensor detects rain it will send a push notification and close the window.
 * Otherwise, the window will open if the temperature is within the limits and the humidity is within range of the target humidity.
 * 
 * Downloads/Installs:
 * This was developed using the Arduino IDE and Virtual Studio Code with the PlatformIO extension.
 * Arduino IDE uses sketches with .ino extensions, PlatformIO uses .cpp files. You should be able to copy over the code in the cpp files into 
 * the .ino file in the Arduino IDE and it should work.
 * - https://www.arduino.cc/en/software
 * - https://code.visualstudio.com/
 * - https://platformio.org/install/ide?install=vscode
 * 
 *************
 * Used guides:
 * - https://learn.adafruit.com/adafruit-tca8418-keypad-matrix-and-gpio-expander-breakout/arduino
 * - https://learn.adafruit.com/adafruit-128x64-oled-featherwing/arduino-code
 * - https://circuitdigest.com/microcontroller-projects/esp32-timers-and-timer-interrupts
 * - https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
 * - https://forum.arduino.cc/t/convert-code-from-stepper-h-to-accelstepper-h/1072838/8
 * 
 * 
 * Github Libraries:
 * - https://github.com/sinricpro/esp8266-esp32-sdk (Sinric Pro)
 * - https://github.com/adafruit/Adafruit_TCA8418 (Keypad)
 * - https://github.com/adafruit/Adafruit_SH110x (OLED Display)
 * - https://github.com/markruys/arduino-DHT (DHT Sensor)
 * - https://github.com/arduino-libraries/Stepper/ (Stepper Motor)
 * - //https://github.com/adafruit/AccelStepper/ (Stepper Motor)?
 * 
 * Other:
 * - https://forums.adafruit.com/viewtopic.php?t=110757 (Graphing on OLED Display initial graph idea since I don't have the part)
 * 
 * 
 * Casey needed ArduinoJson, WebSockets(2.4.0 Markus Sattler), Adafruit_I2CDevice, as additional dependencies not included in the libraries
 * also needed to move weathermonitor.h into IDE
 * 
 * 
 * considerations for extra time:
 * You can set interrupts to trigger on rising or falling edge, or even ONLOW, so I could have the rain sensor wake up
 * the ESP32 and update rain state.
 * Keypad with interrupt???
 * Be able to turn off screen
 * Set timeout lengths for sensors
 * Add ESP32 reset funcitonality from keypad input
 * Add test mode to user options
 * User options structure?
 * 
 * 
 * TODO:
 * - Go over the polling intervals and timeouts and make sure they are using the correct units
 * - Go over the menu system and make sure it is working properly
 * - Go over the settings and make sure they are being used properly
 * 
 * - Make sure the user can use the Serial Monitor to change settings if the menu system doesn't work
 * - Make sure the user can use the Serial Monitor to test the components if the menu system doesn't work
 * 
 * - Make sure the menu system works (test sequence of inputs)
 * 
 * DHT color wires
 * (Green: GND, Yellow: Data, Blue: VCC)
 **/

/*************
 * Libraries *
 *************/

#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #error "ESP32 device required for this project"
#endif

/* Sinric Pro headers */
#include <SinricPro.h>
#include <SinricProTemperaturesensor.h>
#include "WeatherMonitor.h"

/* DHT sensor Library */
#include <DHT.h>

/* Stepper Motor Library */
//#include <Stepper.h>
#include <AccelStepper.h>

/* Keypad Matrix Library */
#include <Adafruit_TCA8418.h>

/* OLED Display Library */
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/* OLED definitions */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// The pins for I2C are defined by the Wire-library. 
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

/* Keypad Definitions */
#define ROWS 4
#define COLS 4

//define the symbols on the buttons of the keypad
char keymap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};


/* Sinric Pro definitions */
#define APP_KEY    "7a7caefc-db9f-4372-b86d-41393f1f74cd"
#define APP_SECRET "8fede556-9e2f-4613-b78b-aeebc5cd2dbb-84d4844f-28c7-4350-9a96-d69387f56bbb"
#define DEVICE_ID  "65386353b1138c00c95e2e88"

/* Wifi Credentials */
#define SSID       "Pixel_7137" //"PSU-IoT" //
#define PASS       "tc9h7msz9rpug8x" //"9SFkew1Hi2HyRANA" //

/* DHT definitions */
#define EVENT_WAIT_TIME   60000           // send event every 60 seconds


/* Stepper Motor definitions */
//FIXME: actually define these

/* Serial Communication rate */
#define BAUD_RATE  115200


/*************
 * Pin Setup *                            //(silkscreen on PCB)
 * ***********/

/* I2C Pins (if you want to relocate the bus) */
#define I2C_SCL 22                        // located on (SCL)
#define I2C_SDA 23                        // located on (SDA)

/* Rain Sensor */
#define rainDigital 13                    //located on (13)

/* DHT */
#define DHT_PIN 15                        // located on (15)

/* Stepper Motor */
#define STEPPER_PIN_1 26                  // located on (A0)
#define STEPPER_PIN_2 25                  // located on (A1)
#define STEPPER_PIN_3 12                  // located on [12]
#define STEPPER_PIN_4 33                  // located on [33]

/* LEDS */
#define POWER_LED 32                      // located on [32]
#define WINDOW_LED 4                      // located on (A5) 
#define TEMP_LED_R 18                      // located on (MOSI)
#define TEMP_LED_G 5                     // located on (SCK)
#define TEMP_LED_B 19                     // located on (MISO) [miso]

/* Alarm Buzzer */
#define ALARM_PIN 21                   // located on (21)


/*************
 * Instances *
 *************/

WeatherMonitor &weatherMonitor = SinricPro[DEVICE_ID];  // make instance of SinricPro device
DHT dht;                                                // make instance of DHT sensor
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_TCA8418 keypad;

/**************
 * Structures *
 **************/

struct MonitorState {
  //Device on/off states
  bool Display_on;    //I2C connection to display
  bool Keypad_on;     //I2C connection to keypad
  bool DHT_on;        //DHT sensor is polled
  bool RainSensor_on; //Rain sensor is polled
  bool Stepper_on;    //Stepper motor is controlled
  
  //Control states
  bool Window_open;   //Window is open
  bool Good_Temp;     //Temperature is within limits
  bool Good_Humidity; //Humidity is within limits
  bool Rain;          //Rain sensor is wet

  //User settings
  bool Use_Target_Temp;     //Use target temperature
  bool Use_Target_Humidity; //Use target humidity
  bool Window_Manual;       //Window is manually controlled
  bool Alarm;               //Alarm is triggered
  bool Sleep;               //Sleep mode is active

  bool Use_SinricPro;       //Use Sinric Pro ?

};

//Start with both I2C devices assumed to be missing, DHT and rain sensor allowed to take readings, stepper motor released,
//window closed, unsafe temperature, unsafe humidity, not currently raining, 
//Using temperature limits, Using humidity limits, window is controlled manually, alarm off, and sleep off
//use sinric pro
static MonitorState Monitor = {
  false, false, true, true, false,
  false, false, false, false,
  false, false, true, false, false,
  true};

struct userSettings{
  // DHT sensor settings
  double upperTemp; //Upper limit for temperature (c)
  double lowerTemp; //Lower limit for temperature (c)

  double upperHumidity; //Upper limit for humidity (%)
  double lowerHumidity; //Lower limit for humidity (%)

  double tempRange; //Range of temperature around target temperature that is acceptable (c)
  double targetTemperature; //Target temperature (c)

  double humidityRange; //Range of humidity around target humidity that is acceptable (%)
  double targetHumidity; //Target humidity (%)

  // Polling settings
  unsigned long DHT_interval;   //Interval at which to poll DHT sensor (ms)
  unsigned long rain_interval;  //Interval at which to poll rain sensor (ms)
  
  unsigned long rain_timeout;   //Timeout for rain sensor (ms)
  unsigned long monitor_timeout;//Timeout for the monitor (ms) //effectively how long to stay awake after last keypad event

  // Other settings
  //type window_position; //Current position of window (manually set to open or closed)
};

/* Start with default values:
 * upperTemp = 27 (c), lowerTemp = 15 (c), 
 * upperHumidity = 60%, lowerHumidity = 40%, 
 * tempRange = 5 (c), targetTemperature = 21 (c), 
 * humidityRange = 10%, targetHumidity = 50%
 * DHT_interval = 60 seconds, rain_interval = 60 seconds,
 * rain_timeout = 60 minutes, monitor_timeout = 15 minutes
 * window_position = closed
 */
static userSettings settings = {
  27.0, 15.0,
  60.0, 40.0,
  5.0, 21.0,
  10.0, 50.0,
  30* 1000, 30 *1000, 60 * 60 * 1000, 15 * 60 * 1000};

/**
 * Main menu
 * -------------
 * Start
 * Settings
 * Test
 * Shutdown : disable all sensors, turn off display and LEDs, release motor, 
 * send final updates to Sinric pro before disconnecting from wifi, deep sleep mode.
 * 
 * 
 *    Start
 *    -------------
 *    Window : Open/Close
 *    Alarm  : On/Off
 *    Sleep  : Go to sleep
 *    Restart: Restart ESP32 TODO: refresh on how software
 *    Exit   : Exit to main menu
 * 
 * 
 *    Settings
 *    -------------
 *    Temperature : Use Target Temperature/Use Temperature Limits -> Set Limits/Set Target Temperature & Range
 *    Humidity : Use Target Humidity/Use Humidity Limits -> Set Limits/Set Target Humidity & Range
 *    Measurement Interval : DHT Interval/Rain Interval
 *    Sleep Mode : How long to wait before turning off screen
 *    Exit : Exit to main menu
 * 
 * 
 *    Test
 *   -------------
 *    Keypad : Display key presses until *#* is pressed
 *    Display : Display test message
 *    Rain Sensor : Display rain sensor value
 *    Stepper Motor : Open/Close window
 *    Alarm : Trigger alarm
 *    LEDs : Toggle LEDs
 *    Exit : Exit to main menu
 * 
 */

// Defining a function pointer type
typedef void (*MenuFunction)();

struct MenuStruct {
    String title;
    std::vector<String> choices;
    int currentSelection;
    
    MenuStruct* parent;
    std::vector<MenuStruct*> children;
    std::vector<MenuFunction> functions;
  };

typedef MenuStruct Menu;
Menu* current_menu = new Menu; //current menu position (used for navigation)

/**************
 * Prototypes *
 **************/

// ToggleController
bool onToggleState(const String& deviceId, const String& instance, bool &state);

// Management functions
void handleTemperaturesensor(void);
void handleRainSensor(void);
void handleKeypad(void);
void handleDisplay(void);
void handleStepper(void);
void handleAlarm(void);
void handleSleep(void);
void handleLEDs(void);
void printWifiStatus(void);


// Menu construction functions
void setParentMenu(Menu* parent_menu, Menu& child_menu);
void addChildMenu(Menu* parent_menu, Menu child_menu);
void addFunction(Menu* menu, MenuFunction function);


// Menu navigation functions
void scrollUp(Menu &menu);
void scrollDown(Menu &menu);
void selectOption(Menu current_menu);
void back();
void printMenu(Menu menu);
void printMenuStatus(Menu* menu);


// Setting menu functions
void changeTempSettings(void);
void changeHumiditySettings(void);
void changeMeasurementInterval(void);
void changeSleepMode(void);
void changeSettings(void);


// Supporting functions
bool check_interval(unsigned long* previousMillis, long interval);
char get_key(void);
void drawGraph(void);
void drawTempGraph(void);
void drawHumidityGraph(void);
void drawMenu(Menu menu);
void drawMenuStatus(Menu menu);
void writeText(String text, int lines);


// Setup functions
void setupSinricPro(void);
void setupWiFi(void);
void setupKeypad(void);
void setupDisplay(void);
void setupMenu(void);
void setupDHT(void);
void setupRainSensor(void);
void setupStepper(void);
void setupAlarm(void);
void setupLEDs(void);


// Debug functions
void testMenu(void);
void testKeypad(void);
void testDisplay(void);
void testRainSensor(void);
void testTemperatureSensor(void);
void testStepper(void);
void testAlarm(void);
void testLEDs(void);

// Simulated input functions
void simulateTemperature(void);
void simulateRain(void);






/*************
 * Variables *
 ***********************************************
 * Global variables to store the device states *
 ***********************************************/

/* Debug Flags */
bool Debug = true; // Set to true to enable debug mode
bool simulatedRain = false; // Set to true to simulate rain
bool simulatedTemperature = false; // Set to true to simulate temperature

// Global variable to store the last known WiFi status
wl_status_t lastWifiStatus = WL_IDLE_STATUS;

// ToggleController
std::map<String, bool> globalToggleStates;

/* DHT device */
bool deviceIsOn;                              // Temeprature sensor on/off state
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)
unsigned long lastEvent = (-EVENT_WAIT_TIME); // last time event has been sent

float temperature;                            // current temperature
float humidity;                               // current humidity



/*************
 *   Timers  *
 * ***********/

/* DHT */
unsigned long DHT_previous_millis = 0;                      // will store last time DHT was checked
const long DHT_sample_interval = settings.DHT_interval;    // interval at which to sample (in milliseconds)

/* LM393 Rain Sensor */
unsigned long LM393_previous_millis = 0;                      // will store last time rain sensor was checked
const long LM393_sample_interval = settings.rain_interval;    // interval at which to sample (in milliseconds)

/* Simulated Menu */
unsigned long menu_previous_millis = 0;                           // will store last time menu was opened/closed
const long menu_refresh_interval = 0.1 * 1000;                      // interval at which to open/close menu (in milliseconds)

/* Simulated Test */
unsigned long test_previous_millis = 0;                           // will store last time test was run
const long test_refresh_interval = 10 * 1000;                      // interval at which to run test (in milliseconds)


/*************
 *   Loop    *
 *************/
void loop() {
  /* Perform Sinric Pro actions*/
  SinricPro.handle();

  /* Check if WiFi status has changed */
  printWifiStatus();

    
  if(check_interval(&LM393_previous_millis, LM393_sample_interval)) {
    //(simulatedRain) ? simulateRain() : testRainSensor();
  }

  if(check_interval(&DHT_previous_millis, DHT_sample_interval)) {
    //(simulatedTemperature) ? simulateTemperature() : testTemperatureSensor();
  }

  /* Check for input from the keypad */
  if (Monitor.Keypad_on){
    Serial.println("Keypad on");
    handleKeypad();
  }
  else{
    // No keypad connected, check if Serial Monitor input is available
    testMenu();
  }

  /* Check for input from the rain sensor */
  if (Monitor.RainSensor_on){
    handleRainSensor();
  }

  /* Measure temperature and humidity. */
  if (Monitor.DHT_on) {
    handleTemperaturesensor();

  }

  /* Display temperature and humidity on the display */
  if (Monitor.Display_on){
    handleDisplay();
  }

  /* Check if window needs to be opened or closed */
  if (Monitor.Stepper_on){
    handleStepper();
  }

  /* Check if alarm needs to be triggered */
  if (Monitor.Alarm){
    //handleAlarm();
  }

  /* Check if sleep mode needs to be activated */
  if (Monitor.Sleep){
    //goToSleep();
  }

  /* Update status LEDs */
  //handleLEDs();


}
  
/*************
 * Functions *
 *************/

void setParentMenu(Menu* parent_menu, Menu& child_menu) {
  child_menu.parent = parent_menu;
}

void addChildMenu(Menu* parent_menu, Menu* child_menu) {
  parent_menu->children.push_back(child_menu);
}

void addFunction(Menu* menu, MenuFunction function) {
  menu->functions.push_back(function);
}

void scrollUp(Menu &menu) {
  menu.currentSelection--;
  if (menu.currentSelection < 0) {
    menu.currentSelection = menu.choices.size() - 1;
  }
}

void scrollDown(Menu &menu) {
  menu.currentSelection++;
  if (menu.currentSelection >= menu.choices.size()) {
    menu.currentSelection = 0;
  }
}

/**
 * selectOption() - Selects the current option in the menu
 * @param current_menu - The current menu layer
 * 
 * This function checks if the current selection is a function or a submenu.
 * If it is a function for the option selected, it calls the function associated
 * with the current selection.
 * If there is no associated function then it is a submenu, and it changes the current menu
 * to the submenu associated with the current selection.
 * 
 * If the current menu has no parent, then it is the main menu and it will not change the current menu.
 * 
 */
void selectOption(Menu*& current_menu) {

  // Check if the current menu exists
  if (current_menu == NULL) {
    Serial.println("No menu selected");
  } 
  else {

    // Check if the current selection has a function associated with it
    if (current_menu->currentSelection < current_menu->functions.size()
    && current_menu->functions[current_menu->currentSelection] != nullptr) {
      // Call the function associated with the current selection
      current_menu->functions[current_menu->currentSelection]();
    }
    // Check if the current selection has a submenu associated with it
    else if (current_menu->currentSelection < current_menu->children.size() 
    && current_menu->children[current_menu->currentSelection] != nullptr) {
      // Change the current menu to the submenu associated with the current selection
      current_menu = current_menu->children[current_menu->currentSelection];
    }
    else {
      Serial.println("No function or submenu associated with current selection");
    }
  }

}


/**
 * @brief Goes back to the parent menu
 * 
 * If the current menu has a parent, then it will change the current menu to the parent menu.
 * If the current menu has no parent, then it is the main menu and it will not change the current menu.
 * 
 */
void back() {
  // Print the fields of the current menu
  //printMenu(*current_menu);

  if (current_menu->parent != NULL) {
    // Inform the user what parent menu they are going back to
    Serial.print("Going back to ");
    Serial.println(current_menu->parent->title);

   // Change the current menu to the parent menu
    current_menu = current_menu->parent;
  }
  else {
    Serial.println("No parent menu");
  }


}

/* User Setting Functions */

/**
 * updateUserSettings() - Updates the user settings
 * @brief Takes keypad input until # is pressed
 * 
 * This function is used to change the user settings stored in the settings struct.
 * It is called by numerous functions to allow the user to change the settings.
 * if the setting is a time, then it converts the input from minutes to milliseconds.
 * 
 * @param setting - The setting to change
 * @param isTime - Whether or not the setting is a time
 * 
 */
template <typename UserSetting>
void updateUserSettings(UserSetting& setting, bool isTime) {
  // Print the current setting
  Serial.print("Current setting: ");
  Serial.println(setting);

  String input = "";

  // Determine if input comes from keypad or Serial Monitor
  if (Monitor.Keypad_on){
    // Get input from keypad
    Serial.println("Enter new setting (press # when done):");
    char key = 0;
    while (key != '#') {
      key = get_key();
      if (key != 0) {
        input += key;
        Serial.print(key);
      }
    }
    Serial.println();
  }
  else{
    // Get input from Serial Monitor
    Serial.println("Enter new setting (press enter when done):");
    while (Serial.available() == 0) {
      // Wait for input
    }
    input = Serial.readStringUntil('\n');
  }

  // Convert input to appropriate type
  if (isTime) {
    setting = strtoul(input.c_str(), NULL, 10) * 60 * 1000;
  }
  else {
    setting = input.toDouble();
  }

  // Print the new setting
  Serial.print("New setting: ");
  Serial.println(setting);

}

/** updateTargetTemperature() - Updates the target temperature and the range of acceptable temperatures
 * @brief Takes keypad input until # is pressed
 *  stores the target temperature and the range of acceptable temperatures in the settings struct.
 * 
 */
void updateTargetTemp() {
  // Print the current target temperature
  Serial.print("Current target temperature: ");
  Serial.println(settings.targetTemperature);

  // Determine if user wants to change the target temperature
  Serial.println("Do you want to change the target temperature?");
  Serial.println("1. Yes");
  Serial.println("2. No");





  updateUserSettings(settings.targetTemperature, false);
}

void updateTempRange() {
  // Print the current temperature range
  Serial.print("Current temperature range: ");
  Serial.println(settings.tempRange);

  // Determine if user wants to change the temperature range
  Serial.println("Do you want to change the temperature range?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.tempRange, false);
}

void updateUpperTemp() {
  // Print the current upper temperature
  Serial.print("Current upper temperature: ");
  Serial.println(settings.upperTemp);

  // Determine if user wants to change the upper temperature
  Serial.println("Do you want to change the upper temperature?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.upperTemp, false);
}

void updateLowerTemp() {
  // Print the current lower temperature
  Serial.print("Current lower temperature: ");
  Serial.println(settings.lowerTemp);

  // Determine if user wants to change the lower temperature
  Serial.println("Do you want to change the lower temperature?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.lowerTemp, false);
}

void updateTargetHumidity() {
  // Print the current target humidity
  Serial.print("Current target humidity: ");
  Serial.println(settings.targetHumidity);

  // Determine if user wants to change the target humidity
  Serial.println("Do you want to change the target humidity?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.targetHumidity, false);
}

void updateHumidityRange() {
  // Print the current humidity range
  Serial.print("Current humidity range: ");
  Serial.println(settings.humidityRange);

  // Determine if user wants to change the humidity range
  Serial.println("Do you want to change the humidity range?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.humidityRange, false);
}

void updateUpperHumidity() {
  // Print the current upper humidity
  Serial.print("Current upper humidity: ");
  Serial.println(settings.upperHumidity);

  // Determine if user wants to change the upper humidity
  Serial.println("Do you want to change the upper humidity?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.upperHumidity, false);
}

void updateLowerHumidity() {
  // Print the current lower humidity
  Serial.print("Current lower humidity: ");
  Serial.println(settings.lowerHumidity);

  // Determine if user wants to change the lower humidity
  Serial.println("Do you want to change the lower humidity?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.lowerHumidity, false);
}

void updateDHTInterval() {
  // Print the current DHT interval
  Serial.print("Current DHT interval: ");
  Serial.print(settings.DHT_interval / (60 * 1000)); // Convert from milliseconds to minutes
  Serial.println("minutes");

  // Determine if user wants to change the DHT interval
  Serial.println("Do you want to change the DHT interval?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.DHT_interval, true);
}

void updateRainInterval() {
  // Print the current rain interval
  Serial.print("Current rain interval: ");
  Serial.print(settings.rain_interval / (60 * 1000)); // Convert from milliseconds to minutes
  Serial.println("minutes");

  // Determine if user wants to change the rain interval
  Serial.println("Do you want to change the rain interval?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.rain_interval, true);
}

void updateRainTimeout() {
  // Print the current rain timeout
  Serial.print("Current rain timeout: ");
  Serial.print(settings.rain_timeout / (60 * 1000)); // Convert from milliseconds to minutes
  Serial.println("minutes");

  // Determine if user wants to change the rain timeout
  Serial.println("Do you want to change the rain timeout?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.rain_timeout, true);
}

void updateMonitorTimeout() {
  // Print the current monitor timeout
  Serial.print("Current monitor timeout: ");
  Serial.print(settings.monitor_timeout / (60 * 1000)); // Convert from milliseconds to minutes
  Serial.println("minutes");

  // Determine if user wants to change the monitor timeout
  Serial.println("Do you want to change the monitor timeout?");
  Serial.println("1. Yes");
  Serial.println("2. No");
  updateUserSettings(settings.monitor_timeout, false);
}


/**
 * changeMeasurementInterval() - Changes the measurement intervals for the DHT sensor and the rain sensor
 * 
 * (needs to be updated to use the I2C display and keypad)
 * 
 * This function allows the user to change the measurement intervals for the DHT sensor and the rain sensor.
 * It is called from the changeSettings() function and when the user selects the measurement interval option
 * from the settings menu.
 * 
 */
void changeMeasurementInterval(){
  printf("Which measurement interval do you want to change?\n");
  printf("1. DHT Interval\n");
  printf("2. Rain Interval\n");
  printf("3. Exit\n");

  int choice = 0;
  scanf("%d", &choice);

  switch(choice){
    case 1:
      printf("Enter DHT interval (ms): ");
      scanf("%lu", &settings.DHT_interval);
      break;
    case 2:
      printf("Enter rain interval (ms): ");
      scanf("%lu", &settings.rain_interval);
      break;
    case 3:
      break;
    default:
      printf("Invalid choice\n");
      break;
  }
}

/**
 * changeSleepMode() - Changes the sleep mode timeout
 * 
 * (needs to be updated to use the I2C display and keypad)
 * 
 * This function allows the user to change the sleep mode timeout.
 * It is called from the changeSettings() function and when the user selects the sleep mode option
 * from the settings menu.
 * 
 * The user can enter the sleep mode timeout in milliseconds
 * 
 */
void changeSleepMode(){
  // Show current sleep mode timeout
  Serial.print("Current sleep mode timeout: ");
  Serial.print(settings.monitor_timeout);
  Serial.println("ms");

  // See if the user wants to change the sleep mode timeout
  Serial.println("Do you want to change the sleep mode timeout?");
  Serial.println("1. Yes");
  Serial.println("2. No");

  int choice = 0;
  scanf("%d", &choice);

  if(choice == 1){
    // Change the sleep mode timeout
    Serial.print("Enter sleep mode timeout (ms): ");
    scanf("%lu", &settings.monitor_timeout);
  }
  else if(choice == 2){
    // Go back to the settings menu
  }
  else{
    Serial.println("Invalid choice");
  }
}

/**
 * changeSettings() - Changes the settings for the monitor
 * 
 * (This is a backup function in case the menu system doesn't work)
 * 
 * This function allows the user to change the settings for the monitor.
 * It allows the user to change the temperature settings, humidity settings,
 * measurement intervals, and sleep mode timeout.
 * 
 * The user can choose to use a target temperature or temperature limits.
 * If they choose to use a target temperature, they can enter the target temperature
 * and the range of acceptable temperatures around the target temperature.
 * If they choose to use temperature limits, they can enter the upper and lower limits
 * for the temperature.
 * 
 * The user can choose to use a target humidity or humidity limits.
 * If they choose to use a target humidity, they can enter the target humidity
 * and the range of acceptable humidity around the target humidity.
 * If they choose to use humidity limits, they can enter the upper and lower limits
 * for the humidity.
 * 
 * The user can change the measurement intervals for the DHT sensor and the rain sensor.
 * 
 * The user can change the sleep mode timeout.
 * 
 * The user can exit the menu.
 * 
 */
void changeSettings(){
  printf("Which setting do you want to change?\n");
  printf("1. Temperature\n");
  printf("2. Humidity\n");
  printf("3. Measurement Interval\n");
  printf("4. Sleep Mode\n");
  printf("5. Exit\n");

  int choice = 0;
  scanf("%d", &choice);

  switch(choice){
    case 1:
      changeTempSettings();
      break;
    case 2:
      changeHumiditySettings();
      break;
    case 3:
      changeMeasurementInterval();
      break;
    case 4:
      changeSleepMode();
      break;
    case 5:
      break;
    default:
      printf("Invalid choice\n");
      break;
  }
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

/* get key press */
char get_key() {
  static unsigned long lastPressTime = 0;  // Time of the last key press
  const unsigned long debounceTime = 50;   // Debounce time in milliseconds


  if (keypad.available()) {
    // Check if enough time has passed since the last key press
    if (millis() - lastPressTime < debounceTime) {
      return 0;  // Not enough time has passed, ignore this key press
    }
    

    //  datasheet page 15 - Table 1
    int k = keypad.getEvent();

    // Mask to identify key press
    bool pressed = k & 0x80;

    // Mask to identify key row and column
    k &= 0x7F;
    k--;
    
    uint8_t row = k / 10;
    uint8_t col = k % 10;

    /*
    if (pressed)
      Serial.print("PRESS\tR: ");
    else
      Serial.print("RELEASE\tR: ");
    Serial.print(row);
    Serial.print("\tC: ");
    Serial.print(col);
    Serial.print(" - ");
    Serial.print(keymap[col][row]);
    Serial.println();
    */

    // Save the time of the key press
    if (pressed) {
      lastPressTime = millis();
    }


    // Band-aid fix for keypad bug? only return key release, not press
    if (!pressed) {
      return keymap[col][row];
    }
    //return keymap[col][row];
  }
  return 0;
}

/* Draw Graph (copied function, verify it works) 
https://forums.adafruit.com/viewtopic.php?t=110757 */
void drawGraph() {
  for (uint8_t i = 1; i < 4; i++) {
    display.drawPixel(6, SCREEN_HEIGHT - SCREEN_HEIGHT / 4 * i, SH110X_WHITE);
  }
  display.drawPixel(27, 62, SH110X_WHITE); 
  display.drawPixel(47, 62, SH110X_WHITE); 
  display.drawPixel(67, 62, SH110X_WHITE); 
  display.drawPixel(87, 62, SH110X_WHITE); 
  display.drawPixel(107, 62, SH110X_WHITE);
  for (uint8_t i = 7; i < SCREEN_WIDTH; i = i + 5) {
    display.drawPixel(i, SCREEN_HEIGHT - SCREEN_HEIGHT / 4, SH110X_WHITE); 
  }
  display.drawFastVLine(7, 0, 63, SH110X_WHITE);
  display.drawFastHLine(7, 63, 120, SH110X_WHITE);
}

/* Draw Temp Graph (copied function, verify it works) 
https://forums.adafruit.com/viewtopic.php?t=110757 */
void drawTempGraph() {

  display.clearDisplay();
  drawGraph();
  display.setCursor(9, 0);
  display.print(F("Temp:"));
  display.print((float) temperature, 1);
  display.println('C');
  display.setCursor(73, 0);
  display.print(F("Cur:"));
  display.print((float) temperature, 1);
  display.println('C');
  display.setCursor(0, 0);
  display.write(24); 
  display.setCursor(0, 8);
  display.print('T'); 
  

}

// ToggleController
bool onToggleState(const String& deviceId, const String& instance, bool &state) {
  Serial.printf("[Device: %s]: State for \"%s\" set to %s\r\n", deviceId.c_str(), instance.c_str(), state ? "on" : "off");
  globalToggleStates[instance] = state;
  return true;
}



/*************
 * Handlers  *
 *************/

//TODO: Add handlers for other devices, rewrite handlers as needed to accomodate function calls properly
//     Better follow the flowchart of events/scheduling for standard operation

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too many events
 * - Get actual temperature and humidity and check if these values are within limits
 * - Compares actual temperature and humidity to last known temperature and humidity
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

  //Check if temperature or humidity exceeds limits
  if(Monitor.Use_Target_Temp){
    //Check if temperature is within range of target temperature
    if(temperature > settings.targetTemperature + settings.tempRange || 
    temperature < settings.targetTemperature - settings.tempRange){
      Monitor.Good_Temp = false;
    }
    else{
      Monitor.Good_Temp = true;
    }
  }
  else{
    //Check if temperature is within limits
    if(temperature > settings.upperTemp || temperature < settings.lowerTemp){
      Monitor.Good_Temp = false;
    }
    else{
      Monitor.Good_Temp = true;
    }
  }

  if(Monitor.Use_Target_Humidity){
    //Check if humidity is within range of target humidity
    if(humidity > settings.targetHumidity + settings.humidityRange || 
    humidity < settings.targetHumidity - settings.humidityRange){
      Monitor.Good_Humidity = false;
    }
    else{
      Monitor.Good_Humidity = true;
    }
  }
  else{
    //Check if humidity is within limits
    if(humidity > settings.upperHumidity || humidity < settings.lowerHumidity){
      Monitor.Good_Humidity = false;
    }
    else{
      Monitor.Good_Humidity = true;
    }
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
 *         - Add rain sensor timeout during rain event (will likely be wet for a while even after rain stops)
 */
void handleRainSensor(){
  /* Determine if enough time has passed since last check-in */
  if (check_interval(&LM393_previous_millis, LM393_sample_interval)) {
    int rainDigitalVal = digitalRead(rainDigital);
    Serial.print("Digitial reading: ");
    Serial.println(rainDigitalVal);

    // Single line flag assignment (if rainDigitalVal is 0, then Monitor.Rain = true, else Monitor.Rain = false)
    //Monitor.Rain = !rainDigitalVal;

    //0 is short - raining
    //1 is open - not raining
    if (rainDigitalVal) {
      Monitor.Rain = false;
      Serial.println("It's not raining!");
    }
    else {
      Monitor.Rain = true;
      Serial.println("It's raining!");
    }
  }
}

/* handleKeypad()
 * - Follows guide: https://learn.adafruit.com/adafruit-tca8418-keypad-matrix-and-gpio-expander-breakout/arduino
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
  // Attempt to print the display, will be set to false if no key is pressed
  bool updateDisplay = true;
  
  // Check if enough time has passed since the last key press
  static unsigned long lastPressTime = 0;  // Time of the last key press
  const unsigned long debounceTime = 50;   // Debounce time in milliseconds

  char key = get_key();

  // If a key is pressed and enough time has passed since the last key press
  if (key && (millis() - lastPressTime > debounceTime)) {
    lastPressTime = millis();  // Update the time of the last key press
  }
  else {
    return; // No key pressed or not enough time has passed since last key press
  }

  // analyze user inputs
  switch (key) {
    
    // Navigation keys
    case 'A':
      Serial.println("Scroll up");
      scrollUp(*current_menu);
      break;
    case 'B':
      Serial.println("Scroll down");
      scrollDown(*current_menu);
      break;
    case 'C':
      Serial.println("Select");
      selectOption(current_menu);
      break;
    case 'D':
      Serial.println("Back");
      back();
      break;

    // Other keys
    case '*':
      Serial.println("Star"); //what should this do?
      break;
    case '#':
      Serial.println("Hash"); //what should this do?
      break;
      
    // if a number is pressed, print it to Serial/Send it to the menu
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      Serial.println(key);
      updateDisplay = false;
      //add key press to string (they typed a number);
      break;

    // if no key is pressed, do nothing
    default:
      updateDisplay = false;
      break;
  }

  // Print the current menu status to the serial monitor
  if (updateDisplay) {
    printMenuStatus(current_menu);
    updateDisplay = false;
  }

  //TODO: Add menu system, probably using discrete functions instead of member functions [scrollUP() instead of menu.scrollUp()]
}  

/* handleDisplay()
 * - TODO: - Display current temperature and humidity on the display
 *         - Other functionality not yet implemented
 *  
 */
void handleDisplay(){


  //TODO: Add display functionality

  // If the temperature or humidity has changed, update the display
  if (temperature != lastTemperature || humidity != lastHumidity) {
    Serial.println("Found measurements:");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" Humidity: ");
    Serial.println(humidity);
    Serial.println("Displaying measurements:");

    //Send measurements to display
    //drawTempGraph();
  }

  
}

/* handleStepper() */
void handleStepper(){
  if (Monitor.Window_Manual){
    //TODO: Add manual control functionality
  }
  else{
    if (Monitor.Window_open){
      //stepper.moveTo(OPEN);
    }
    else{
      //stepper.moveTo(CLOSED);
    }
  }


}

/* handleAlarm() */
void handleAlarm(){
  //TODO: Add alarm functionality
}

/* handleSleep() */
void handleSleep(){
  //TODO: Add sleep functionality
}

/* handleLEDs() */
void handleLEDs(){
  //TODO: Add LED functionality
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

  // Restore state of toggle device "toggleInstance1" when device is connected
  SinricPro.restoreDeviceStates(true);
};

void setupWiFi() {
  
  WiFi.setSleep(false); 
  WiFi.setAutoReconnect(true);
  
  WiFi.begin(SSID, PASS);
  Serial.printf("[WiFi]: Connecting to %s", SSID);
}

void setupKeypad() {
  // Initialize the TCA8418 with I2C addr 0x34
  if (!keypad.begin(TCA8418_DEFAULT_ADDR, &Wire)){
    Serial.println("Keypad not found!");
    Monitor.Keypad_on = false; //TODO: better user feedback (Visual and/or audible?)
    return;
  }
  Monitor.Keypad_on = true; //Initialized successfully
  Serial.println("Keypad initialized!");

  // configure the size of the keypad matrix.
  // all other pins will be inputs
  keypad.matrix(ROWS, COLS);

  // flush the internal buffer
  keypad.flush();
}

void setupDisplay() {
  //https://learn.adafruit.com/adafruit-128x64-oled-featherwing/arduino-code
  // Initialize display with I2C addr 0x3D
  if(!display.begin(SCREEN_ADDRESS)) {
    Serial.println(F("Display not found"));
    Monitor.Display_on = false; //TODO: better user feedback (Visual and/or audible?)
    return;
  }
  Monitor.Display_on = true; //Initialized successfully
  Serial.println("Display initialized!");

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
}

/***********************************
 * setupMenu() - Sets up the menu system
 * 
 * This function sets up the menu system. It creates the menus and connects them together.
 * It also sets the current menu to the main menu.
 * 
 ***********************************
 * Main menu
 * -------------
 * Start
 * Settings
 * Test
 * Shutdown : disable all sensors, turn off display and LEDs, release motor, 
 * send final updates to Sinric pro before disconnecting from wifi, deep sleep mode.
 * 
 * 
 *    Start
 *    -------------
 *    Window : Open/Close
 *    Alarm  : On/Off
 *    Sleep  : Go to sleep
 *    Restart: Restart ESP32 TODO: refresh on how software
 *    Exit   : Exit to main menu
 * 
 * 
 *    Settings
 *    -------------
 *    Temperature : Use Target Temperature/Use Temperature Limits -> Set Limits/Set Target Temperature & Range
 *    Humidity : Use Target Humidity/Use Humidity Limits -> Set Limits/Set Target Humidity & Range
 *    Measurement Interval : DHT Interval/Rain Interval
 *    Sleep Mode : How long to wait before turning off screen
 *    Exit : Exit to main menu
 * 
 * 
 *    Test
 *   -------------
 *    Keypad : Display key presses until *#* is pressed
 *    Display : Display test message
 *    Rain Sensor : Display rain sensor value
 *    Stepper Motor : Open/Close window
 *    Alarm : Trigger alarm
 *    LEDs : Toggle LEDs
 *    Exit : Exit to main menu
 * 
 */
void setupMenu() {
  Menu* start_menu = new Menu{
    "Start",
    {"Window", "Alarm", "Sleep", "Restart", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* settings_menu = new Menu{
    "Settings",
    {"Temperature", "Humidity", "Measurement Interval", "Sleep Mode", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* test_menu = new Menu{
    "Test",
    {"Keypad", "Display", "Rain Sensor", "Stepper Motor", "Alarm", "LEDs", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* main_menu = new Menu{
    "Main Menu",
    {"Start", "Settings", "Test", "Shutdown"},
    0,
    nullptr,
    {start_menu, settings_menu, test_menu, nullptr},
    {nullptr, nullptr, nullptr, /*TODO: Add shutdown function */}
  };

  Menu* TempMenu = new Menu{
    "Temperature",
    {"Use Target Temperature", "Use Temperature Limits", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* targetTempMenu = new Menu{
    "Target Temperature",
    {"Target Temperature", "Temperature Range", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* tempLimitsMenu = new Menu{
    "Temperature Limits",
    {"Lower Temperature", "Upper Temperature", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* humidityMenu = new Menu{
    "Humidity",
    {"Use Target Humidity", "Use Humidity Range", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* targetHumidityMenu = new Menu{
    "Target Humidity",
    {"Target Humidity", "Humidity Range", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* humidityLimitsMenu = new Menu{
    "Humidity Limits",
    {"Lower Humidity", "Upper Humidity", "Exit"},
    0,
    nullptr,
    {},
    {}
  };
  
  Menu* IntervalMenu = new Menu{
    "Measurement Interval",
    {"DHT Interval", "Rain Interval", "Monitor Interval", "Exit"},
    0,
    nullptr,
    {},
    {}
  };


  //* Connect menus */ 
  /* 1st level menus */

  // start menu
  setParentMenu(main_menu, *start_menu);
  addFunction(start_menu, handleStepper);
  addFunction(start_menu, handleAlarm);
  addFunction(start_menu, handleSleep);
  addFunction(start_menu, nullptr);  //TODO: Add restart function
  addFunction(start_menu, back);

  // settings menu
  setParentMenu(main_menu, *settings_menu);
  addChildMenu(settings_menu, TempMenu);
  addChildMenu(settings_menu, humidityMenu);
  addChildMenu(settings_menu, IntervalMenu);

  addFunction(settings_menu, nullptr);
  addFunction(settings_menu, nullptr);
  addFunction(settings_menu, nullptr);
  addFunction(settings_menu, changeSleepMode);
  addFunction(settings_menu, back);

  // test menu
  setParentMenu(main_menu, *test_menu);
  addFunction(test_menu, testKeypad);
  addFunction(test_menu, testDisplay);
  addFunction(test_menu, testRainSensor);
  addFunction(test_menu, testStepper);
  addFunction(test_menu, testAlarm);
  addFunction(test_menu, testLEDs);
  addFunction(test_menu, back);

  /* 2nd level menus */
  // temperature menu
  setParentMenu(settings_menu, *TempMenu);
  addChildMenu(TempMenu, targetTempMenu);
  addChildMenu(TempMenu, tempLimitsMenu);
  addFunction(TempMenu, nullptr);
  addFunction(TempMenu, nullptr); //TODO: Add set temperature range function
  addFunction(TempMenu, back);

  // humidity menu
  setParentMenu(settings_menu, *humidityMenu);
  addChildMenu(humidityMenu, targetHumidityMenu);
  addChildMenu(humidityMenu, humidityLimitsMenu);
  addFunction(humidityMenu, nullptr); //TODO: Add set humidity function
  addFunction(humidityMenu, nullptr); //TODO: Add set humidity range function
  addFunction(humidityMenu, back);

  // measurement interval menu
  setParentMenu(settings_menu, *IntervalMenu);
  addFunction(IntervalMenu, updateDHTInterval);
  addFunction(IntervalMenu, updateRainInterval);
  addFunction(IntervalMenu, updateMonitorTimeout);
  addFunction(IntervalMenu, back);

  /* 3rd level menus */
  // target temperature menu
  setParentMenu(settings_menu, *targetTempMenu);  // Go back to settings menu
  addFunction(targetTempMenu, updateTargetTemp); // Set target temperature function
  addFunction(targetTempMenu, updateTempRange); // Set temperature range function
  addFunction(targetTempMenu, back);

  // temperature limits menu
  setParentMenu(settings_menu, *tempLimitsMenu);  // Go back to settings menu
  addFunction(tempLimitsMenu, updateLowerTemp); // Set lower temperature function
  addFunction(tempLimitsMenu, updateUpperTemp); // Set upper temperature function
  addFunction(tempLimitsMenu, back);

  // target humidity menu
  setParentMenu(settings_menu, *targetHumidityMenu);  // Go back to settings menu
  addFunction(targetHumidityMenu, updateTargetHumidity); // Set target humidity function
  addFunction(targetHumidityMenu, updateHumidityRange); // Set humidity range function
  addFunction(targetHumidityMenu, back);

  // humidity limits menu
  setParentMenu(settings_menu, *humidityLimitsMenu);  // Go back to settings menu
  addFunction(humidityLimitsMenu, updateLowerHumidity); // Set lower humidity function
  addFunction(humidityLimitsMenu, updateUpperHumidity); // Set upper humidity function
  addFunction(humidityLimitsMenu, back);

  // Sanity check Menus
  std::vector<Menu*> menus = {main_menu, start_menu, settings_menu, test_menu, 
  TempMenu, targetTempMenu, tempLimitsMenu, 
  humidityMenu, targetHumidityMenu, humidityLimitsMenu, 
  IntervalMenu};

  for (Menu* menu : menus) {
    //Serial.println(menu->title + " Sanity Check:");
    //printMenu(*menu);
  }

  
  // Set current menu to main menu
  current_menu = main_menu;
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(5000); // give me time to bring up serial monitor
  
  // Rain sensor input
  pinMode(rainDigital, INPUT);

  // Status LEDs
  pinMode(POWER_LED, OUTPUT);
  pinMode(WINDOW_LED, OUTPUT);
  
  // RGB LED
  pinMode(TEMP_LED_R, OUTPUT);
  pinMode(TEMP_LED_G, OUTPUT);
  pinMode(TEMP_LED_B, OUTPUT);

  // Stepper motor
  //stepper.setMaxSpeed(1000);
  //stepper.setAcceleration(1000);
  //pinMode(STEPPER_MOTOR_PIN_1, OUTPUT);
  //pinMode(STEPPER_MOTOR_PIN_2, OUTPUT);
  //pinMode(STEPPER_MOTOR_PIN_3, OUTPUT);
  //pinMode(STEPPER_MOTOR_PIN_4, OUTPUT);

  // Alarm
  pinMode(ALARM_PIN, OUTPUT);


  // Initializations
  setupDisplay();
  setupKeypad();
  setupMenu();
  setupWiFi();
  setupSinricPro();

  // Initial setup complete, send push notification to alert user
  sendPushNotification("Device is online!");
  
  // Initial readings
  //handleRainSensor();   // check rain sensor
  //handleTemperaturesensor(); // check temperature sensor
}

/********* 
 * Test Functions *
 *********/

void testKeypad(){  
  if (keypad.available() > 0) {
    //  datasheet page 15 - Table 1
    int k = keypad.getEvent();
    bool pressed = k & 0x80;
    k &= 0x7F;
    k--;
    uint8_t row = k / 10;
    uint8_t col = k % 10;

    if (pressed)
      Serial.print("PRESS\tR: ");
    else
      Serial.print("RELEASE\tR: ");
    Serial.print(row);
    Serial.print("\tC: ");
    Serial.print(col);
    Serial.print(" - ");
    Serial.print(keymap[col][row]);
    Serial.println();
  }
}

void testDisplay(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Hello World!");
  display.display();
}

void testRainSensor(){
  
  int rainDigitalVal = digitalRead(rainDigital);
  Serial.print("Digitial reading: ");
  Serial.println(rainDigitalVal);
}

void testTemperatureSensor(){
  
  // Check the status of the sensor
  Serial.println(dht.getStatusString());
  
  temperature = dht.getTemperature();          // get actual temperature in °C
  humidity = dht.getHumidity();                // get actual humidity
  
  Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);

  //Serial.print("Temperature: ");
  //Serial.print(temperature);
  //Serial.print(" Humidity: ");
  //Serial.println(humidity);
  //Serial.println("Displaying measurements:");
}

void testStepper(){
  /*
  if (stepper.distanceToGo() == 0)
    {
        // Random change to speed, position and acceleration
        // Make sure we dont get 0 speed or accelerations
        delay(1000);
        stepper.moveTo(rand() % 200);
        stepper.setMaxSpeed((rand() % 200) + 1);
        stepper.setAcceleration((rand() % 200) + 1);
    }
    stepper.run();
    */
}

void testAlarm(){
  //TODO: Add alarm functionality
  Serial.println("Triggering alarm for 5 seconds");
  digitalWrite(ALARM_PIN, HIGH);
  delay(5000);
  digitalWrite(ALARM_PIN, LOW);
  Serial.println("Alarm off");
  
}

/**
 * @brief Runs a test of the LEDs
 * 
 * Turns on the Power LED for 3 seconds before turning it off.
 * Turns on the Status LED for 3 seconds before turning it off.
 * 
 * Sets Temperature RGB LED to white for 3 seconds.
 * Then turns LED red, blue, green for 3 seconds each.
 * Then cycles through the color spectrum for 5 seconds.
 * Then turns off the Temperature LED.
 * 
 */
void testLEDs(){
  //TODO: Add LED functionality
  Serial.println("Testing LEDs");

  Serial.println("Power LED on");
  digitalWrite(POWER_LED, HIGH);
  delay(3000);
  Serial.println("Power LED off");
  digitalWrite(POWER_LED, LOW);


  Serial.println("Status LED on");
  digitalWrite(WINDOW_LED, HIGH);
  delay(3000);
  Serial.println("Status LED off");
  digitalWrite(WINDOW_LED, LOW);


  Serial.println("Temperature LED on");
  // White
  analogWrite(TEMP_LED_R, 255);
  analogWrite(TEMP_LED_G, 255);
  analogWrite(TEMP_LED_B, 255);
  delay(3000);

  Serial.println("Temperature LED: Too Hot/Dry");
  // Red
  analogWrite(TEMP_LED_R, 255);
  analogWrite(TEMP_LED_G, 0);
  analogWrite(TEMP_LED_B, 0);
  delay(3000);

  Serial.println("Temperature LED: Too Cold/Wet");
  // Blue
  analogWrite(TEMP_LED_R, 0);
  analogWrite(TEMP_LED_G, 0);
  analogWrite(TEMP_LED_B, 255);
  delay(3000);

  Serial.println("Temperature LED: Just Right");
  // Green
  analogWrite(TEMP_LED_R, 0);
  analogWrite(TEMP_LED_G, 255);
  analogWrite(TEMP_LED_B, 0);
  delay(3000);
  
  // Timing to cycle through the color spectrum in 5 seconds
  int totalSteps = 6 * 256; // 6 color transitions, 256 steps each
  int totalTime = 5000; // Total time for the color sweep in milliseconds
  int delayTime = totalTime / totalSteps; // Time to delay between each step

  Serial.println("Temperature LED: Color Sweep");
  // Sweep through the color spectrum

  for (int i = 0; i < 256; i++) {
    // Red to Yellow
    analogWrite(TEMP_LED_R, 255);
    analogWrite(TEMP_LED_G, i);
    analogWrite(TEMP_LED_B, 0);
    delay(delayTime);
  }

  for (int i = 0; i < 256; i++) {
    // Yellow to Green
    analogWrite(TEMP_LED_R, 255 - i);
    analogWrite(TEMP_LED_G, 255);
    analogWrite(TEMP_LED_B, 0);
    delay(delayTime);
  }

  for (int i = 0; i < 256; i++) {
    // Green to Cyan
    analogWrite(TEMP_LED_R, 0);
    analogWrite(TEMP_LED_G, 255);
    analogWrite(TEMP_LED_B, i);
    delay(delayTime);
  }

  for (int i = 0; i < 256; i++) {
    // Cyan to Blue
    analogWrite(TEMP_LED_R, 0);
    analogWrite(TEMP_LED_G, 255 - i);
    analogWrite(TEMP_LED_B, 255);
    delay(delayTime);
  }

  for (int i = 0; i < 256; i++) {
    // Blue to Magenta
    analogWrite(TEMP_LED_R, i);
    analogWrite(TEMP_LED_G, 0);
    analogWrite(TEMP_LED_B, 255);
    delay(delayTime);
  }

  for (int i = 0; i < 256; i++) {
    // Magenta to Red
    analogWrite(TEMP_LED_R, 255);
    analogWrite(TEMP_LED_G, 0);
    analogWrite(TEMP_LED_B, 255 - i);
    delay(delayTime);
  }

  // Turn off Temperature LED
  Serial.println("Temperature LED off");
  analogWrite(TEMP_LED_R, 0);
  analogWrite(TEMP_LED_G, 0);
  analogWrite(TEMP_LED_B, 0);


  Serial.println("LED test complete");
}

void testMenu(){
  // Will print the current menu status to the serial monitor on the first run
  static bool updateDisplay = true;

  // Check if there's available data on the serial port
  if (Serial.available() > 0) {
    // Read the input
    char input = Serial.read();

    // The display probably needs to be updated
    updateDisplay = true;
    
    // Perform the corresponding action
    switch (input) {
      case 'a':
      case 'A':
        scrollUp(*current_menu);
        break;
      case 'b':
      case 'B':
        scrollDown(*current_menu);
        break;
      case 'c':
      case 'C':
        selectOption(current_menu);
        break;
      case 'd':
      case 'D':
        back();
        break;
      default:
        // Invalid input
        updateDisplay = false;
        break;
    }
  }

  // Check if the display needs to be updated
  if(updateDisplay){
    // Print the current menu
    //printMenu(*current_menu);
    printMenuStatus(current_menu);

    updateDisplay = false;
  }
  

}

void printWifiStatus() {
  wl_status_t currentWifiStatus = WiFi.status();

  if (currentWifiStatus != lastWifiStatus) {
    switch (currentWifiStatus) {
      case WL_CONNECTED:
        Serial.println("WiFi connected");
        break;
      case WL_DISCONNECTED:
        Serial.println("WiFi disconnected");
        break;
      // Add more cases if you want to handle other status codes
    }

    lastWifiStatus = currentWifiStatus;
  }
}


/**
 * @brief Simulates rain occuring in a random interval between 1 and 5 minutes
 * 
 * This function simulates rain occuring in a random interval between 1 and 5 minutes.
 * When called, it will roll a random number between 1 and 5 and print that it is raining only
 * if the number is 1.
 * Updates the rain flag in the Monitor struct.
 * 
 */
void simulateRain(){
  // 1 in 5 chance of rain
  int raining = ((rand() % 5 + 1) == 1);

  // Inform user if it's raining through serial monitor
  if(raining){
    Serial.println("It's raining!");
  }
  else{
    Serial.println("It's not raining!");
  }

  // Set rain flag
  Monitor.Rain = raining;


}

/**
 * @brief Generates random temperature and humidity values to simulate readings
 * 
 * This function simulates temperature and humidity readings with random values.
 * The temperature will be a random value between 10C and 30C.
 * The humidity will be a random value between 30% and 70%.
 * 
 * 
 * 
 */
void simulateTemperature(){
  //simulate random temperature reading between 10C and 30C
  temperature = rand() % 20 + 10;

  //simulate random humidity reading between 30% and 70%
  humidity = rand() % 40 + 30;
  
}

/**
 * printMenuStatus() - Prints the current menu status to the serial monitor
 * @param menu - The current menu layer
 * 
 * This function prints the current menu status to the serial monitor.
 * It prints the menu title, the current selection, and the choices.
 * It also prints a ">" next to the current selection.
 * 
 * The menu passed in has the format:
 * 
 * Menu {
 * String title,
 * vector<String> choices,
 * int currentSelection,
 * Menu* parent,
 * vector<Menu*> children,
 * vector<MenuFunction> functions
 * }
 * 
 * The menu is printed in the following format:
 * 
 * Menu Title: <title>
 * Current Selection: <currentSelection>
 * Choices:
 * <choice 1>
 * <choice 2>
 * ...
 * <choice n>
 * 
 * If the menu passed in is null, then it prints "No menu selected"
 * 
 * 
 */
void printMenuStatus(Menu* menu) {
  if (menu != NULL) {
    Serial.println("Menu Title: " + menu->title);
    Serial.println("Current Selection: " + String(menu->currentSelection));
    Serial.println("Choices:");
    for (int i = 0; i < menu->choices.size(); i++) {
      Serial.print(i == menu->currentSelection ? ">" : " ");
      Serial.println(menu->choices[i]);
    }
  } else {
    Serial.println("No menu selected");
  }
}

void printMenu(Menu menu) {
  Serial.println("Title: " + menu.title);
  Serial.println("Choices:");
  for (const auto& choice : menu.choices) {
    Serial.println("  " + choice);
  }
  Serial.println("Current Selection: " + String(menu.currentSelection));
  if (menu.parent != nullptr) {
    Serial.println("Parent: " + menu.parent->title);
  } else {
    Serial.println("Parent: None");
  }
  Serial.println("Children:");
  for (const auto& child : menu.children) {
    if (child != nullptr) {
      Serial.println("  " + child->title);
    }
  }
  Serial.println("Functions:");
  for (const auto& function : menu.functions) {
    if (function != nullptr) {
      Serial.println("  Function exists");
    } else {
      Serial.println("  No function");
    }
  }
}
