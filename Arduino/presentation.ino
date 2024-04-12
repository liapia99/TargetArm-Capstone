
#include "LiquidCrystal_I2C.h"
#include <Wire.h>

int red_led = 13;
int green_led = 8;
LiquidCrystal_I2C lcd(0x27,  16, 2); 

void setup() {
   lcd.init();
   lcd.backlight();
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    printDistance(inputString);
  }
}

void printDistance(String input) {
  int distance = input.toInt(); // Convert the input string to an integer

  if (distance > 12) {
    digitalWrite(red_led, LOW); // Turn off red LED
    digitalWrite(green_led, HIGH); // Turn on green LED
    lcd.clear(); // Clear the LCD
    lcd.setCursor(0, 0);
    lcd.print("Object Detected!");
    lcd.setCursor(0, 1);
    lcd.print("Distance: ");
    lcd.print(distance);
  } else {
    digitalWrite(green_led, LOW); // Turn off green LED
    digitalWrite(red_led, HIGH); // Turn on red LED
    lcd.clear(); // Clear the LCD
    lcd.setCursor(0, 0);
    lcd.print("Object Detected!");
    lcd.setCursor(0, 1);
    lcd.print("Distance: ");
    lcd.print(distance);
  }
}
