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
 * Set timeout lengths for sensors (X)
 * Add ESP32 reset funcitonality from keypad input
 * Add test mode to user options (X)
 * User options structure? (X)
 * 
 * 
 * TODO:
 * - Go over the polling intervals and timeouts and make sure they are using the correct units
 * - Go over the menu system and make sure it is working properly
 * - Go over the settings and make sure they are being used properly
 * 
 * - Make sure the user can use the Serial Monitor to change settings if the menu system doesn't work (X)
 * - Make sure the user can use the Serial Monitor to test the components if the menu system doesn't work (X)
 * 
 * - Make sure the menu system works (test sequence of inputs) (X)
 * 
 * 
 * - Move the menu system to a separate file (header and cpp)
 * - Refactor the Serial monitor output to use a function which can be switched between Serial Monitor and OLED Display
 * 
 * - Add option to view current settings
 * - Add option to reset settings to default
 * 
 * - Ensure that strings are not too long for the OLED display
 * - Hsve dedicated area on OLED display for messages?
 * 
 * - Fix the graphing on the OLED display
 * 
 * - Add toggle to view current temperature and humidity and/or graph
 * 
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
#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSerif9pt7b.h>

//#include <Adafruit_SH110X.h>
#include <Adafruit_SSD1306.h>




/***********
 * Defines *
 **********/

/* OLED definitions */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// The pins for I2C are defined by the Wire-library. 
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Test Display definitions
#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16

#define XPOS   0 // Indexes into the 'icons' array in testanimate function below
#define YPOS   1
#define DELTAY 2
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

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
Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
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
  bool Window_open;   //Window is open (true) or closed (false)
  bool Good_Temp;     //Temperature is within limits
  bool Good_Humidity; //Humidity is within limits
  bool Rain;          //Rain sensor is wet
  bool Display_Menu;  //Display menu on OLED display (supressed when monitoring)

  //User settings
  bool Use_Target_Temp;     //Use target temperature
  bool Use_Target_Humidity; //Use target humidity
  bool Window_Manual;       //Window is manually controlled
  bool Alarm;               //Alarm is triggered
  bool Silent;              //Alarm is silent
  bool Sleep;               //Sleep mode is active

  bool Use_SinricPro;       //Use Sinric Pro ?

};

//Start with both I2C devices assumed to be missing, DHT and rain sensor allowed to take readings, stepper motor released,
//window closed, unsafe temperature, unsafe humidity, not currently raining, and menu displayed
//Using temperature limits, Using humidity limits, window is controlled manually, alarm off, and sleep off
//use sinric pro
 MonitorState Monitor = {
  false, false, true, true, false,
  false, false, false, false, true,
  false, false, true, false, false, false,
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
  bool too_hot; //Temperature is too hot
  bool too_cold; //Temperature is too cold
  bool too_humid; //Humidity is too high
  bool too_dry; //Humidity is too low
  bool window_open; //Window is manually set to open
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
 userSettings settings = {
  27.0, 15.0,
  60.0, 40.0,
  5.0, 21.0,
  10.0, 50.0,
  30* 1000, 30 *1000, 60 * 60 * 1000, 15 * 60 * 1000,
  false, false, false, false, false};

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
 *    Monitor: (display current temperature and humidity until key is pressed)
 *    Window : Open/Close
 *    Alarm  : On/Off
 *    Sleep  : Go to sleep
 *    Restart: Restart ESP32
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
void watchSensors(void);
void toggleAlarm(void);


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
void viewSettings(void);
void resetSettings(void);
void restartESP32(void);

// Keypad input functions
void updateTargetTemp(void);
void updateTempRange(void);
void updateUpperTemp(void);
void updateLowerTemp(void);
void updateTargetHumidity(void);
void updateHumidityRange(void);
void updateUpperHumidity(void);
void updateLowerHumidity(void);
void updateDHTInterval(void);
void updateRainInterval(void);
void updateRainTimeout(void);
void updateMonitorTimeout(void);


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
void setRGBLED(int color);
void updateTempFlags(void);

// Simulated input functions
void simulateTemperature(void);
void simulateRain(void);

// Oled display functions
void testdrawstyles(void);
void printMenu(Menu* menu);
void printUserMessage(String message);

void clear_line(int line);
void clear_menu_lines(void);

void displayMessage(String message);
void print_long_message(String message);


// Mega draw test functions
void megaDrawTest(void);
void testdrawline(void);
void testdrawrect(void);
void testfillrect(void);
void testdrawcircle(void);
void testfillcircle(void);
void testdrawroundrect(void);
void testfillroundrect(void);
void testdrawtriangle(void);
void testfilltriangle(void);
void testdrawchar(void);
void testdrawstyles(void);
void testscrolltext(void);
void testdrawbitmap(void);
void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h);






/*************
 * Variables *
 ***********************************************
 * Global variables to store the device states *
 ***********************************************/

/* Debug Flags */
bool Debug = true; // Set to true to enable debug mode
bool UpdateDisplay = true; // Set to true to update the display
bool UpdateTempDisplay = false; // Set to true to update the temperature display
bool keypress = false; // Set to true when a key is pressed

// Global variable to store the last known WiFi status
wl_status_t lastWifiStatus = WL_IDLE_STATUS;

// Global pointer to the current menu
Menu* current_menu = new Menu; // Start with an empty menu

// Global string to hold messages to be displayed on the OLED display
String userMessage = "";

// ToggleController
std::map<String, bool> globalToggleStates;

/* DHT device */
bool deviceIsOn;                              // Temeprature sensor on/off state
float lastTemperature;                        // last known temperature (for compare)
float lastHumidity;                           // last known humidity (for compare)

float temperature;                            // current temperature
float humidity;                               // current humidity



/*************
 *   Timers  *
 * ***********/

/* Handle timers*/

/* DHT */
unsigned long DHT_previous_millis = 0;                      // will store last time DHT was checked
const long DHT_sample_interval = settings.DHT_interval;    // interval at which to sample (in milliseconds)

/* LM393 Rain Sensor */
unsigned long LM393_previous_millis = 0;                      // will store last time rain sensor was checked
const long LM393_sample_interval = settings.rain_interval;    // interval at which to sample (in milliseconds)

/* Serial Menu */
unsigned long menu_previous_millis = 0;                           // will store last time menu was opened/closed
const long menu_refresh_interval = 0.1 * 1000;                      // interval at which to open/close menu (in milliseconds)

/* Display */
unsigned long display_previous_millis = 0;                           // will store last time display was updated
const long display_refresh_interval = 0.1 * 1000;                      // interval at which to update display (in milliseconds)

/* Alarm */
unsigned long alarm_previous_millis = 0;                            // will store last time alarm was triggered
const long alarm_refresh_interval = 5 * 1000;                       // interval at which to trigger alarm (in milliseconds)
const long alarm_tone_length = 10;                                        // length of alarm tone (in milliseconds)

/* Test timers */

/* Unassigned test */
unsigned long test_previous_millis = 0;                           // will store last time test was run
const long test_refresh_interval = 10 * 1000;                      // interval at which to run test (in milliseconds)

/* Alarm */
unsigned long test_alarm_previous_millis = 0;                            // will store last time alarm was triggered
const long test_alarm_refresh_interval = 5 * 1000;                       // interval at which to trigger alarm (in milliseconds)
const long tone_length = 10;                                        // length of alarm tone (in milliseconds)



/*************
 *   Loop    *
 *************/
void loop() {
  /* Perform Sinric Pro actions*/
  SinricPro.handle();

  /* Check if WiFi status has changed */
  printWifiStatus();
  
  /* Check for input from the user */
  handleKeypad();

  /* update the display if needed */
  handleDisplay();

  /* Check for input from the temperature sensor */
  handleTemperaturesensor();

  /* Check for input from the rain sensor */
  if (Monitor.RainSensor_on){
    handleRainSensor();
  }

  /* Measure temperature and humidity. */
  if (Monitor.DHT_on) {
    handleTemperaturesensor();    
  }

  /* Check if window needs to be opened or closed */
  if (Monitor.Stepper_on){
    handleStepper();
  }

  /* Check if alarm needs to be triggered */
  if (Monitor.Alarm){
    handleAlarm();
  }

  /* Check if sleep mode needs to be activated */
  if (Monitor.Sleep){
    //goToSleep();
  }

  /* Update status LEDs */
  handleLEDs();


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
  
  if(Monitor.Display_on){
    // Display the current setting
    String message = "Current setting: " + String(setting);
    //printUserMessage(message);
  }
  else{
    // Print the current setting
    Serial.print("Current setting: ");
    Serial.println(setting);
  }

  String input = "";

  // Determine if input comes from keypad or Serial Monitor
  if (Monitor.Keypad_on){
    // Get input from keypad
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Enter new setting");
    display.println("(press # when done):");
    display.display();



    Serial.println("Enter new setting (press # when done):");
    char key = 0;
    while (key != '#') {
      key = get_key();
      if (key != 0 && key != '#') {
        input += key;
        displayMessage((String) input);
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
  String message = "New setting: " + String(setting);
  if (isTime) {
    message += " min";
  }
  displayMessage(message);
}

/** updateTargetTemperature() - Updates the target temperature and the range of acceptable temperatures
 * @brief Takes keypad input until # is pressed
 *  stores the target temperature and the range of acceptable temperatures in the settings struct.
 * 
 */
void updateTargetTemp() {
  // Print the current target temperature
  String message = "Currently: " + String(settings.targetTemperature) + " C\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the target temperature

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the target temperature
    updateUserSettings(settings.targetTemperature, false);
  }
  else if (key == '2') {
    // Do not update the target temperature
    Serial.println("Target temperature not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateTempRange() {
  String message = "Currently: " + String(settings.tempRange) + " C\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the temperature range

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the temperature range
    updateUserSettings(settings.tempRange, false);
  }
  else if (key == '2') {
    // Do not update the temperature range
    Serial.println("Temp range not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateUpperTemp() {
  String message = "Currently: " + String(settings.upperTemp) + " C\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the upper temperature limit

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the upper temperature limit
    updateUserSettings(settings.upperTemp, false);
  }
  else if (key == '2') {
    // Do not update the upper temperature limit
    Serial.println("upper temp not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateLowerTemp() {
  String message = "Currently: " + String(settings.lowerTemp) + " C\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the lower temperature limit

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the lower temperature limit
    updateUserSettings(settings.lowerTemp, false);
  }
  else if (key == '2') {
    // Do not update the lower temperature limit
    Serial.println("lower temp not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateTargetHumidity() {
  String message = "Currently: " + String(settings.targetHumidity) + " %\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the target humidity

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the target humidity
    updateUserSettings(settings.targetHumidity, false);
  }
  else if (key == '2') {
    // Do not update the target humidity
    Serial.println("Target humidity not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateHumidityRange() {
  String message = "Currently: " + String(settings.humidityRange) + " %\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the humidity range

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the humidity range
    updateUserSettings(settings.humidityRange, false);
  }
  else if (key == '2') {
    // Do not update the humidity range
    Serial.println("Humidity range not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateUpperHumidity() {
  String message = "Currently: " + String(settings.upperHumidity) + " %\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the upper humidity limit

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the upper humidity limit
    updateUserSettings(settings.upperHumidity, false);
  }
  else if (key == '2') {
    // Do not update the upper humidity limit
    Serial.println("upper humidity not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateLowerHumidity() {
  String message = "Currently: " + String(settings.lowerHumidity) + " %\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the lower humidity limit

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if (key == '1') {
    // Update the lower humidity limit
    updateUserSettings(settings.lowerHumidity, false);
  }
  else if (key == '2') {
    // Do not update the lower humidity limit
    Serial.println("lower humidity not changed");
  }
  else {
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateDHTInterval() {
  // Print the current DHT interval
  String message = "Current DHT interval: " + String(settings.DHT_interval / (60 * 1000)) + " minutes\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the DHT interval

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0) {
    // Wait for input
    key = get_key();
  }

  if(key == '1'){
    // Update the DHT interval
    updateUserSettings(settings.DHT_interval, true);
  }
  else if(key == '2'){
    // Do not update the DHT interval
    Serial.println("DHT interval not changed");
  }
  else{
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateRainInterval() {
  // Print the current rain interval
  String message = "Current rain interval: " + String(settings.rain_interval / (60 * 1000)) + " minutes\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the rain interval

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0){
    // Wait for input
    key = get_key();
  }

  if(key == '1'){
    // Update the rain interval
    updateUserSettings(settings.rain_interval, true);
  }
  else if(key == '2'){
    // Do not update the rain interval
    Serial.println("Rain interval not changed");
  }
  else{
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateRainTimeout() {
  // Print the current rain timeout
  String message = "Current rain timeout: " + String(settings.rain_timeout / (60 * 1000)) + " minutes\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the rain timeout

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();

  char key = get_key();
  while (key == 0){
    // Wait for input
    key = get_key();
  }

  if(key == '1'){
    // Update the rain timeout
    updateUserSettings(settings.rain_timeout, true);
  }
  else if(key == '2'){
    // Do not update the rain timeout
    Serial.println("Rain timeout not changed");
  }
  else{
    // Invalid input
    Serial.println("Invalid input");
  }
}

void updateMonitorTimeout() {
  // Print the current monitor timeout
  String message = "Current monitor timeout: " + String(settings.monitor_timeout / (60 * 1000)) + " minutes\n";
  displayMessage(message);

  // bad blocking delay FIXME: interrupts?
  delay(3000);

  // Determine if user wants to change the monitor timeout

  display.clearDisplay();
  display.setCursor(0,0);
  message = "Change?\n";
  display.println(message);

  message = "(1. Yes / 2. No)\n";
  display.println(message);
  display.display();
  
  char key = get_key();
  while (key == 0){
    // Wait for input
    key = get_key();
  }

  if(key == '1'){
    // Update the monitor timeout
    updateUserSettings(settings.monitor_timeout, true);
  }
  else if(key == '2'){
    // Do not update the monitor timeout
    Serial.println("Monitor timeout not changed");
  }
  else{
    // Invalid input
    Serial.println("Invalid input");
  }
}

void restartESP32() {
  Serial.println("Restarting ESP32...");
  ESP.restart();
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

    keypress = false; // Reset keypress flag
    // Save the time of the key press
    if (pressed) {
      lastPressTime = millis();
    }


    // Band-aid fix for keypad bug? only return key release, not press
    if (!pressed) {
      if(keymap[col][row] == '*'){
        Monitor.Silent = !Monitor.Silent;
        keypress = true;  // Set keypress flag on key release
      }

      return keymap[col][row];
    }
    //return keymap[col][row];
  }
  return 0;
}

/* Draw Graph (copied function, verify it works) 
https://forums.adafruit.com/viewtopic.php?t=110757 */
void drawGraph() {

  // clear display
  display.clearDisplay();

  /* draw graph */
  // Vertical line on the left
  for (uint8_t i = 1; i < 4; i++) {
    display.drawPixel(6, SCREEN_HEIGHT - (SCREEN_HEIGHT / 4) * i, SSD1306_WHITE);
  }


  display.drawPixel(27, 62, SSD1306_WHITE); 
  display.drawPixel(47, 62, SSD1306_WHITE); 
  display.drawPixel(67, 62, SSD1306_WHITE); 
  display.drawPixel(87, 62, SSD1306_WHITE); 
  display.drawPixel(107, 62, SSD1306_WHITE);

  // Horizontal line on the bottom
  for (uint8_t i = 7; i < SSD1306_WHITE; i = i + 5) {
    display.drawPixel(i, SCREEN_HEIGHT - SCREEN_HEIGHT / 4, SSD1306_WHITE); 
  }

  display.drawFastVLine(7, 0, 63, SSD1306_WHITE);
  display.drawFastHLine(7, 63, 120, SSD1306_WHITE);

  // draw buffer
  display.display();
}

/* Draw Temp Graph (copied function, verify it works) 
https://forums.adafruit.com/viewtopic.php?t=110757 */
void drawTempGraph() {

  // Draw graph
  drawGraph();

  // hold for 5 seconds
  delay(5000);

  // clear display
  display.clearDisplay();

  // draw text
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

  // Draw the readings
  display.display();

  // hold for 5 seconds
  delay(5000);
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

/* Watch Sensors() */
void watchSensors() {
Monitor.Display_Menu = false; // Suppress menu function calls (should be re-enabled when key is pressed)

// Constantly looks for keypad/Serial input, returns to main menu on key press
// Suppresses menu function calls (should be re-enabled when key is pressed)

if(!UpdateTempDisplay){
  return;
}

if(UpdateTempDisplay){
  // Temporarily suppress keypresses on display update
  keypress = false;

  display.clearDisplay();
  // Otherwise, display the current temperature and humidity
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);

  String message = "Temp        Humid";
  display.setCursor(0, 0);
  display.println(message);

  display.setTextSize(2);

  message = String((int) temperature) + "C " + String((int)humidity) + "%";
  
  display.println(message);
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.display();
}

  //if keypress occurs after display update, return to main menu
  if(keypress){
    displayMessage("Returning to main menu");
    Monitor.Display_Menu = true;
    return;

  }

}

/* handleTemperatatureSensor()
 * - Checks if Temperaturesensor is turned on
 * - Checks if time since last event > EVENT_WAIT_TIME to prevent sending too many events
 * - Get actual temperature and humidity and check if these values are within limits
 * - Compares actual temperature and humidity to last known temperature and humidity
 */
void handleTemperaturesensor() {
  
  temperature = dht.getTemperature();          // get actual temperature in °C
//  temperature = dht.getTemperature() * 1.8f + 32;  // get actual temperature in °F
  humidity = dht.getHumidity();                // get actual humidity

  if (isnan(temperature) || isnan(humidity)) { // reading failed... 
    Serial.printf("DHT reading failed!\r\n");  // print error message
    return;                                    // try again next time
  } 

  // Otherwise, update temp flags
  updateTempFlags();
  
  // If sinric pro is connected, send event
  if(SinricPro.isConnected()){
    bool success = weatherMonitor.sendTemperatureEvent(temperature, humidity); // send event
    if (success) {  // if event was sent successfuly, print temperature and humidity to serial
      Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
    } else {  // if sending event failed, print error message
      Serial.printf("Something went wrong...could not send Event to server!\r\n");
      return;
    }
  }


  // Check if either value changed
  if (temperature != lastTemperature || humidity != lastHumidity) {
    // Tell the display it may update
    UpdateTempDisplay = true;
    
    // Update the last known temperature and humidity
    lastTemperature = temperature;  // save actual temperature for next compare
    lastHumidity = humidity;        // save actual humidity for next compare
  }
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
 *
 * Is one of primary handlers, should be called every loop
 * Will check if keypad is connected, if not, will use Serial Monitor
 * If keypad is connected, will check for key presses (debounced)
 * If a key is pressed, will call appropriate function, setting flags to update display
 * 
 * 
 * - Follows guide: https://learn.adafruit.com/adafruit-tca8418-keypad-matrix-and-gpio-expander-breakout/arduino
 *
 */
void handleKeypad(){

  // Check if keypad is missing
  if (!Monitor.Keypad_on) {
    if (Debug) Serial.println("Keypad not connected");
    
    // Check for input from Serial Monitor
    testMenu();
    return;
  }


  // Check if enough time has passed since the last key press
  static unsigned long lastPressTime = 0;  // Time of the last key press
  const unsigned long debounceTime = 50;   // Debounce time in milliseconds

  // Check if a key has been pressed
  char key = get_key();

  // If a key is pressed and enough time has passed since the last key press
  if (key && (millis() - lastPressTime > debounceTime)) {
    lastPressTime = millis();  // Update the time of the last key press
  }
  else {
    keypress = false; // Reset keypress flag
    return; // No key pressed or not enough time has passed since last key press
  }

  // analyze user inputs
  switch (key) {
    
    // Navigation keys
    case 'A':
      Serial.println("Scroll up");
      scrollUp(*current_menu);
      UpdateDisplay = true;
      break;
    case 'B':
      Serial.println("Scroll down");
      scrollDown(*current_menu);
      UpdateDisplay = true;
      break;
    case 'C':
      Serial.println("Select");
      selectOption(current_menu);
      UpdateDisplay = true;
      break;
    case 'D':
      Serial.println("Back");
      back();
      UpdateDisplay = true;
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

      Monitor.Alarm = false; // Stop the alarm
      keypress = true; // Set keypress flag
      Monitor.Display_Menu = true; // Allow menu function calls
      if (Debug) Serial.println(key);
      //add key press to string (they typed a number);
      break;

    // if no key is pressed, do nothing
    default:
      if (Debug) Serial.println("No key pressed");
      break;
  }

}  

/* handleDisplay()
 * - TODO: - Display current temperature and humidity on the display
 *         - Other functionality not yet implemented
 *  
 */
void handleDisplay(){

  // Check if we are monitoring the sensors (ignoring the menu)
  if(!Monitor.Display_Menu){
    watchSensors();

    /* Print temp and humid as such
      Temp  Humid   // 1 line tall
      XX.xC XX.x%   // 3 lines tall
      */

    // Don't display the menu
    return;
  }


  // Check if the display needs to be updated (if the menu has changed)
  if(UpdateDisplay){
    // Print the current menu over I2C
    if(Monitor.Display_on){
      printMenu(current_menu);
    }

    // Print the current menu status to the serial monitor
    if(!Monitor.Display_on || Debug){
      printMenuStatus(current_menu);
    }
    UpdateDisplay = false;
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
  if(get_key() != 0){
    Monitor.Alarm = false;
    noTone(ALARM_PIN);
    return;
  }

  if(Monitor.Silent){
    noTone(ALARM_PIN);
    return;
  }

  bool increasing = true;
  int frequency = 400;
  
  if (millis() - alarm_previous_millis >= alarm_tone_length) {
    // Save the last time a tone was played
    alarm_previous_millis = millis();

    // Play a tone on the buzzer
    tone(ALARM_PIN, frequency);

    // Update the frequency
    if (increasing) {
      frequency++;
      if (frequency == 800) {
        increasing = false;
      }
    } else {
      frequency--;
      if (frequency == 400) {
        increasing = true;
      }
    }
  }
    
}

/* handleSleep() */
void handleSleep(){
  //TODO: Add sleep functionality
}

/* handleLEDs() */
void handleLEDs(){
  updateTempFlags();

  // Check if Window is open
  if (Monitor.Window_open){
    // True if Window is open
    digitalWrite(WINDOW_LED, HIGH);
  }
  else{
    // False if Window is closed
    digitalWrite(WINDOW_LED, LOW);
  }

  // Check the status of the temperature sensor
  if(Monitor.Good_Humidity && Monitor.Good_Temp){
    setRGBLED(2); // Green
    
  }
  else{
    // Determine what is out of limits
    if (settings.too_hot && settings.too_dry){
      setRGBLED(7); // Orange
    }
    else if (settings.too_cold && settings.too_humid){
      setRGBLED(6); // magenta
    }
    else if(settings.too_hot){
      setRGBLED(1); // Red
    }
    else if(settings.too_cold){
      setRGBLED(3); // Blue
    }
    else if(settings.too_dry){
      setRGBLED(4); // Yellow
    }
    else if(settings.too_humid){
      setRGBLED(5); // Cyan
    }
    else{
      Serial.println("Error: No limits set");
    }
    
  }

  // Any other LEDs?

  
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
  //https://learn.adafruit.com/adafruit-oled-featherwing/usage
  // Initialize display with I2C addr 0x3D
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Display not found"));
    Monitor.Display_on = false; //TODO: better user feedback (Visual and/or audible?)
    return;
  }
  
  Monitor.Display_on = true; //Initialized successfully
  Serial.println("Display initialized!");

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds
  display.clearDisplay(); // Clear the buffer

  //display.setRotation(1);
  display.setTextSize(1);
  //display.setFont(&FreeSerif9pt7b); //TODO: Find a smaller font (currently default is best)
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,0);

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
  // Create menus

  Menu* start_menu = new Menu{
    "Start",
    {"Monitor", "Window", "Alarm", "Sleep", "Restart", "Exit"},
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
    {"Keypad", "Display", "DHT", "Rain Sensor", "Stepper Motor", "Alarm", "LEDs", "Exit"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* main_menu = new Menu{
    "Main Menu",
    {"Start Monitor", "Settings", "Test systems", "Shutdown"},
    0,
    nullptr,
    {start_menu, settings_menu, test_menu, nullptr},
    {nullptr, nullptr, nullptr, /*TODO: Add shutdown function */}
  };

  Menu* TempMenu = new Menu{
    "Temperature",
    {"Use Target", "Use Limits", "Back"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* targetTempMenu = new Menu{
    "Target Temperature",
    {"Target Temperature", "Allowed Range", "Back"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* tempLimitsMenu = new Menu{
    "Temperature Limits",
    {"Lower Limit", "Upper Limit", "Back"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* humidityMenu = new Menu{
    "Humidity",
    {"Use Target", "Use Limits", "Back"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* targetHumidityMenu = new Menu{
    "Target Humidity",
    {"Target Humidity", "Allowed Range", "Back"},
    0,
    nullptr,
    {},
    {}
  };

  Menu* humidityLimitsMenu = new Menu{
    "Humidity Limits",
    {"Lower Limit", "Upper Limit", "Back"},
    0,
    nullptr,
    {},
    {}
  };
  
  Menu* IntervalMenu = new Menu{
    "Measurement Interval",
    {"DHT Interval", "Rain Interval", "Monitor Interval", "Back"},
    0,
    nullptr,
    {},
    {}
  };


  //* Connect menus */ 
  /* 1st level menus */

  // start menu
  setParentMenu(main_menu, *start_menu);
  addFunction(start_menu, watchSensors);
  addFunction(start_menu, handleStepper);
  addFunction(start_menu, toggleAlarm);
  addFunction(start_menu, handleSleep);
  addFunction(start_menu, restartESP32);  //TODO: Add restart function
  addFunction(start_menu, back);

  // settings menu
  setParentMenu(main_menu, *settings_menu);
  addChildMenu(settings_menu, TempMenu);
  addChildMenu(settings_menu, humidityMenu);
  addChildMenu(settings_menu, IntervalMenu);

  addFunction(settings_menu, nullptr);  // Move to temperature menu
  addFunction(settings_menu, nullptr);  // Move to humidity menu
  addFunction(settings_menu, nullptr);  // Move to measurement interval menu
  addFunction(settings_menu, nullptr);  // TODO: Implement sleep mode
  addFunction(settings_menu, back);

  // test menu
  setParentMenu(main_menu, *test_menu);
  addFunction(test_menu, testKeypad);
  addFunction(test_menu, testDisplay);
  addFunction(test_menu, testTemperatureSensor);
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

  // DHT sensor
  dht.setup(DHT_PIN); // data pin 2

  // Status LEDs
  pinMode(POWER_LED, OUTPUT);
  digitalWrite(POWER_LED, HIGH);

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

  delay(5000);
  printMenu(current_menu);  // 5 is the number of items, 0 is the menu index
}



/********* 
 * Test Functions *
 *********/

void testKeypad(){  

  if(Monitor.Keypad_on == false){
    displayMessage("Keypad not initialized!");
    return;
  }

  // print the key pressed until *#* is pressed
  char last3keys[3] = {0, 0, 0};
  while(last3keys[0] != '*' || last3keys[1] != '#' || last3keys[2] != '*') {
    
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

      // Store the last 3 keys released
      if (!pressed) {
        last3keys[0] = last3keys[1];
        last3keys[1] = last3keys[2];
        last3keys[2] = keymap[col][row];
      }
    }//end if keypad.available()
  }//end while
  
}

void testDisplay(){
  if(Monitor.Display_on == false){
    Serial.println("Display not initialized!");
    return;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Hello World!");
  // Test components if connected (DHT and Rain)
  yield();
  display.display();
  
  // hold for 5 seconds
  delay(5000);

  megaDrawTest();
}

void testRainSensor(){
  
  int rainDigitalVal = digitalRead(rainDigital);

  String message = "Rain Sensor: ";
  if (rainDigitalVal) {
    message += "Dry";
  }
  else {
    message += "Wet";
  }
  displayMessage(message);
}

void testTemperatureSensor(){
  // Check minimum sampling period (DHT11: 1000ms, DHT22: 2000ms)
  //Serial.print("Minimum sampling period: ");
  //Serial.println(dht.getMinimumSamplingPeriod());


  // Check the status of the sensor
  if(strcmp(dht.getStatusString(), "OK") != 0){
    Serial.println(dht.getStatusString());
  }

  temperature = dht.getTemperature();          // get actual temperature in °C
  humidity = dht.getHumidity();                // get actual humidity
  

  String message = "Temp: " + String(temperature) + "C\n";
  displayMessage(message);
  
  delay(3000);
  message = "Hum: " + String(humidity) + "%";
  displayMessage(message);

  //Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
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
  // Increase frequency

  test_alarm_previous_millis = millis();
  while (millis() - test_alarm_previous_millis < 5000){
    for (int i = 400; i < 800; i++) {
      tone(ALARM_PIN, i);
      delay(tone_length); // Short delay between tones
    }
    
    // Decrease frequency
    for (int i = 800; i > 400; i--) {
      tone(ALARM_PIN, i);
      delay(tone_length); // Short delay between tones
    }
  }
  noTone(ALARM_PIN);
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

  Serial.println("Double time now");
  for(int color=0; color <8; color++){
    setRGBLED(color);
    delay(1000);
  }
}

void testMenu(){
  // Check if there's available data on the serial port
  if (Serial.available() > 0) {
    // Read the input
    char input = Serial.read();
    
    // Perform the corresponding action
    switch (input) {
      case 'a':
      case 'A':
        scrollUp(*current_menu);
        UpdateDisplay = true;
        break;
      case 'b':
      case 'B':
        scrollDown(*current_menu);
        UpdateDisplay = true;
        break;
      case 'c':
      case 'C':
        selectOption(current_menu);
        UpdateDisplay = true;
        break;
      case 'd':
      case 'D':
        back();
        UpdateDisplay = true;
        break;
      default:
        // Invalid input
        UpdateDisplay = false;
        break;
    }
  }

  // No input from Serial Monitor
  else {
    UpdateDisplay = false;
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



/*********
 * Utils *    (Or unassigned functions)
 *********/

void toggleAlarm(){
  Monitor.Silent = !Monitor.Silent;
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

void setRGBLED(int color){
  switch(color) {
    case 0: //off
      analogWrite(TEMP_LED_R, 0);
      analogWrite(TEMP_LED_G, 0);
      analogWrite(TEMP_LED_B, 0);
      break;
    case 1: //red (too hot)
      analogWrite(TEMP_LED_R, 255);
      analogWrite(TEMP_LED_G, 0);
      analogWrite(TEMP_LED_B, 0);
      break;
    case 2: //green (just right)
      analogWrite(TEMP_LED_R, 0);
      analogWrite(TEMP_LED_G, 255);
      analogWrite(TEMP_LED_B, 0);
      break;
    case 3: //blue (too cold)
      analogWrite(TEMP_LED_R, 0);
      analogWrite(TEMP_LED_G, 0);
      analogWrite(TEMP_LED_B, 255);
      break;
    case 4: //yellow (too dry)
      analogWrite(TEMP_LED_R, 255);
      analogWrite(TEMP_LED_G, 255);
      analogWrite(TEMP_LED_B, 0);
      break;
    case 5: //cyan (too humid)
      analogWrite(TEMP_LED_R, 0);
      analogWrite(TEMP_LED_G, 255);
      analogWrite(TEMP_LED_B, 255);
      break;
    case 6: //magenta (too cold and too humid)
      analogWrite(TEMP_LED_R, 255);
      analogWrite(TEMP_LED_G, 0);
      analogWrite(TEMP_LED_B, 255);
      break;
    case 7: //white (too hot and too dry)
      analogWrite(TEMP_LED_R, 255);
      analogWrite(TEMP_LED_G, 255);
      analogWrite(TEMP_LED_B, 255);
      break;
  
    default:
      Serial.println("Invalid color");
      break;
  }
}

void updateTempFlags() {
  // Serial.println("Updating temperature flags");
  if(Monitor.Use_Target_Temp){
    //Check if temperature is within range of target temperature
    if (temperature > settings.targetTemperature + settings.tempRange) {
      // Temperature is too hot
      settings.too_hot = true;
      settings.too_cold = false;
      Monitor.Good_Temp = false;
    } else if (temperature < settings.targetTemperature - settings.tempRange) {
      // Temperature is too cold
      settings.too_cold = true;
      settings.too_hot = false;
      Monitor.Good_Temp = false;
    } else {
      // Temperature is good
      settings.too_hot = false;
      settings.too_cold = false;
      Monitor.Good_Temp = true;
    }
  }
  else{
    if (temperature > settings.upperTemp) {
      // Temperature is too hot
      settings.too_hot = true;
      settings.too_cold = false;
      Monitor.Good_Temp = false;
    } else if (temperature < settings.lowerTemp) {
      // Temperature is too cold
      settings.too_cold = true;
      settings.too_hot = false;
      Monitor.Good_Temp = false;
    } else {
      // Temperature is good
      settings.too_hot = false;
      settings.too_cold = false;
      Monitor.Good_Temp = true;
    }
  }

  if(Monitor.Use_Target_Humidity){
      //Check if humidity is within range of target humidity
      if(humidity > settings.targetHumidity + settings.humidityRange) {
        // Humidity is too high
        settings.too_humid = true;
        settings.too_dry = false;
        Monitor.Good_Humidity = false;
      } else if(humidity < settings.targetHumidity - settings.humidityRange) {
        // Humidity is too low
        settings.too_dry = true;
        settings.too_humid = false;
        Monitor.Good_Humidity = false;
      } else {
        // Humidity is good
        settings.too_humid = false;
        settings.too_dry = false;
        Monitor.Good_Humidity = true;
      }
  } else {
      //Check if humidity is within limits
      if(humidity > settings.upperHumidity) {
        // Humidity is too high
        settings.too_humid = true;
        settings.too_dry = false;
        Monitor.Good_Humidity = false;
      } else if(humidity < settings.lowerHumidity) {
        // Humidity is too low
        settings.too_dry = true;
        settings.too_humid = false;
        Monitor.Good_Humidity = false;
      } else {
        // Humidity is good
        settings.too_humid = false;
        settings.too_dry = false;
        Monitor.Good_Humidity = true;
      }
  }
  
  // Update Alarm if necessary
  if (settings.too_hot || settings.too_cold || settings.too_humid || settings.too_dry) {
    Monitor.Alarm = true;
  } else {
    Monitor.Alarm = false;
  }

}


/****************
 * Loop Rewrite *
 ***************/

void loop2() {
  /* Status updates*/
  // WiFi
  printWifiStatus();
  SinricPro.handle();

  // update LEDS
  handleLEDs();

  // update alarm
  handleAlarm(); 

  // update stepper
  handleStepper();

  /* handle I/O */
  // weather inputs
  handleRainSensor();   // check rain sensor (LM393)
  handleTemperaturesensor(); // check temperature sensor (DHT)

  // Deal with user, NEEDS TO BE REWRITEN
  handleKeypad();
  handleDisplay();


}


void printMenu(Menu* menu) {
  clear_menu_lines();
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK); 

  // Determine the top visible menu item
  int topItem = max( 0, menu->currentSelection - 1);  // show one item above and below the current selection

  // Print the menu on the left side
  display.setCursor(0, 0);
  
  // Print the menu options below the title
  for (int i = topItem; i < min(topItem + 3, (int) menu->choices.size()); i++) {  // Display 3 items, leave 4th line for messages
    if (i == menu->currentSelection) {
      // Highlight the selected option
      display.print("> ");
    }
    display.println(menu->choices[i]);
  }

  // Print the message on the 4th line
  //userMessage = "Hello World!";
  //displayuserMessage(userMessage);

  display.display();
}

void printUserMessage(const String message) {
  clear_line(4);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);

  // Record the message to the serial monitor
  Serial.print("Message: ");
  Serial.println(message);

  // Print the message on the 4th line
  display.setCursor(0, 24);  // 24 is the y coordinate of the 4th line (75% of 32)
  
  // break the message into words so it fits on the screen, check if it fits on the 4th line
  // Print as much of the message as possible on the 4th line, then wait 3 seconds before processing the rest
  // characters are 5 pixels wide + 1 pixel space between characters
  String word = "";
  int wordWidth = 0;
  int x = 0;
  for (int i = 0; i < message.length(); i++) {
    char c = message.charAt(i);
    if (c == ' ') {
      // Print the word if it fits on the 4th line
      if (x + wordWidth <= 128) {
        display.print(word);
        display.print(" ");
        x += wordWidth + 6; // 6 is the width of a space
      } else {
        // The word doesn't fit on the 4th line, so print the message so far and wait 3 seconds
        display.display();
        delay(3000);
        
        // Clear the display and reset the cursor
        clear_line(4);
        display.setCursor(0, 24);
        x = 0;
      }

      // Reset the word
      word = "";
      wordWidth = 0;
    } else {
      // Add the character to the word
      word += c;
      wordWidth += 6;
    }
  }
  
  //Print the last word if it fits on the 4th line
if (word.length() > 0) {
  if (x + wordWidth > 128) {
    // The word doesn't fit on the line, so print the message so far and wait 3 seconds
    display.display();
    delay(3000);
    
    // Clear the display and reset the cursor
    clear_line(4);
    display.setCursor(0, 24);
    x = 0;
  }

  // Print the last word
  display.print(word);
}

// Show the last word
display.display();


}

void clear_line(int line) {
  switch(line) {
    case 1:
      display.setCursor(0, 0);
      break;
    case 2:
      display.setCursor(0, 8);
      break;
    case 3:
      display.setCursor(0, 16);
      break;
    case 4:
      display.setCursor(0, 24);
      break;
    default:
      Serial.println("Invalid line number");
      break;
  }
  for (int i = 0; i < SCREEN_WIDTH / 6; i++) { // display width of 128 pixels and character width of 6 pixels
    display.print(' ');
  }
  display.display();
}

void clear_menu_lines() {
  clear_line(1);
  clear_line(2);
  clear_line(3);
}


void displayMessage(String message) {
  // determine if message should be passed to Serial Monitor or OLED display (or both)
  if (Monitor.Display_on) {
    // Display message on OLED display
    printUserMessage(message);  //TODO: Add message to queue/break up message into multiple lines
    // if message > SCREEN_WIDTH / 6, set flag to scroll message
  }
  
  if(!Monitor.Display_on || Debug){
    // Display message on Serial Monitor
    Serial.println(message);
  }

}

void print_long_message(String message) {
  // overwrites the current menu with a long message
  Serial.println("Printing long message");
  Serial.println(message);

  // clear the display
  display.clearDisplay();

  // Calculate the number of lines needed to display the message
  int NumCharsPerLine = SCREEN_WIDTH / 10; // 6 is the width of a character
  int numLines = ceil(message.length() / NumCharsPerLine); // 6 is the width of a character

  Serial.print("Number of lines: ");
  Serial.println(numLines);
  
  int start = 0;  // start of the substring to print
  int end = 0;    // end of the substring to print
  int temp_end = 0; // temporary placeholder for the end of the substring to print

  // if the message is too long to fit on the screen, print the first part of the message and wait 3 seconds
  // When printing substrings, make sure to split on newlines and on the last space before the end of the line
  for(int i = 0; i < numLines; i++) {
    // Calculate the end of the substring to print assuming best case scenario
    end = min(start + NumCharsPerLine, (int) message.length());
    
    // Check if the end of the substring needs to be adjusted
    if(end < message.length()) {
      // If there is a newline in the substring, move the end to the newline
      temp_end = message.indexOf('\n', end);
      if (temp_end != -1) {
        // move the end to the newline, removing the newline character
        end = temp_end-1;
      }

      // Otherwise, if the end of the substring is in the middle of a word,
      //move the end to the last space before the end of the line
      else if(message.charAt(end) != ' ') {
        temp_end = message.lastIndexOf(' ', end);
        if(temp_end != -1) {
          end = temp_end;
        }
      }
    }// end if(end < message.length())

    // Serial.print("Start: ");
    // Serial.print(start);
    // Serial.print(" End: ");
    // Serial.println(end);

    // Print the substring
    //display.setCursor(0, (i % 4) * 8);  // 8 is the height of a character
    display.println(message.substring(start, end));
    Serial.println(message.substring(start, end));
    display.display();

    if(i % 4 == 3) {
      // Wait 3 before clearing the display
      delay(3000);
      display.clearDisplay();
      display.setCursor(0, 0);
    }

    // Update the start of the substring to print
    start = end;
  }
   

  // Make sure the last part of the message is printed
  display.display();
}



/******************
 * Graphics tests *
 *****************/

void megaDrawTest() {
  testdrawline();      // Draw many lines

  testdrawrect();      // Draw rectangles (outlines)

  testfillrect();      // Draw rectangles (filled)

  testdrawcircle();    // Draw circles (outlines)

  testfillcircle();    // Draw circles (filled)

  testdrawroundrect(); // Draw rounded rectangles (outlines)

  testfillroundrect(); // Draw rounded rectangles (filled)

  testdrawtriangle();  // Draw triangles (outlines)

  testfilltriangle();  // Draw triangles (filled)

  testdrawchar();      // Draw characters of the default font

  testdrawstyles();    // Draw 'stylized' characters

  testscrolltext();    // Draw scrolling text

  testdrawbitmap();    // Draw a small bitmap image

  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);

  testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT); // Animate bitmaps
}


void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}
void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  unsigned long animationStart = millis();
  while (millis() - animationStart < 10*1000){ // Loop for 10 seconds...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }

  // Clear the screen one last time to clear animation leftovers.
  display.clearDisplay();

}


