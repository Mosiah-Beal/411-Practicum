// Menu.cpp
#include "Menu.h"


// Global pointer to the current menu
extern Menu* current_menu; 


/* Prototype definitions */
// Menu creation functions
void setParentMenu(Menu* parent_menu, Menu& child_menu) {
  child_menu.parent = parent_menu;
}

void addChildMenu(Menu* parent_menu, Menu* child_menu) {
  parent_menu->children.push_back(child_menu);
}

void addFunction(Menu* menu, MenuFunction function) {
  menu->functions.push_back(function);
}

// Menu navigation functions
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
void selectOption(Menu* current_menu) {

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


// Menu utility functions
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
