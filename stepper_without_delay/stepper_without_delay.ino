
//www.elegoo.com
//2018.10.25

/*
  Stepper Motor Control - one revolution

  This program drives a unipolar or bipolar stepper motor.
  The motor is attached to digital pins 8 - 11 of the Arduino.

  The motor should revolve one revolution in one direction, then
  one revolution in the other direction.

*/

#include <Stepper.h>

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 15;         // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 5000;           // interval at which to blink (milliseconds)

// initialize the stepper library on pins 8 through 11:
// For ESP32, pins are 33, 27, 12, 13
Stepper myStepper(stepsPerRevolution, 33, 27, 12, 13);

void setup() {
  myStepper.setSpeed(rolePerMinute);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
    // Check to see if it's time to rotate the stepper motor
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save the last time the motor was rotated
    previousMillis = currentMillis;
} 
// Step one revolution in one direction
    Serial.println("clockwise");
  myStepper.step(stepsPerRevolution);
  
// Step one revolution in the other direction
  Serial.println("counterclockwise");
  myStepper.step(-stepsPerRevolution);
  }
