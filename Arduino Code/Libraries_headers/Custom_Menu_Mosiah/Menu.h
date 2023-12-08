// Menu.h

#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include <vector>


/** Example Menu system:
 * 
 * 
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

// Defining a function pointer type for menu functions
typedef void (*MenuFunction)();

// The menu structure will either go to a child menu or call a function
// 
struct MenuStruct {
    String title;   // Title of the menu
    std::vector<String> choices;    // List of choices user can make
    int currentSelection;   // Index of current selection
    
    MenuStruct* parent; // Pointer to parent menu
    std::vector<MenuStruct*> children;  // List of pointers to children menus
    std::vector<MenuFunction> functions;    // List of functions to call when a choice is made
  };

// Defining a Menu type
typedef MenuStruct Menu;


/************
 * Prototypes
 ************/

// Menu construction functions
void setParentMenu(Menu* parent_menu, Menu& child_menu);
void addChildMenu(Menu* parent_menu, Menu* child_menu);
void addFunction(Menu* menu, MenuFunction function);

// Menu navigation functions
void scrollUp(Menu &menu);
void scrollDown(Menu &menu);
void selectOption(Menu* current_menu);
void back();

// Menu utility functions
void printMenu(Menu menu);
void printMenuStatus(Menu* menu);



#endif // MENU_H