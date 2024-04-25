#include "LiquidCrystal_I2C.h"
#include <Wire.h>

int red_led = 13;
int green_led = 8;
int connector = 12;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();
  
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(connector, OUTPUT);
  
  Serial.begin(115200);   // Communication with Python    // Initialize Serial2 for communication with another Arduino
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    if (inputString.equals("Time Start")) {
      digitalWrite(connector, HIGH);  // set pin 12 high
    }
    
    int commaIndex = inputString.indexOf(',');
    if (commaIndex != -1) {
      String distanceString = inputString.substring(0, commaIndex);
      String zString = inputString.substring(commaIndex + 1);
      float distance = distanceString.toFloat();
      int z = zString.toInt();
      printDistance(distance, z);  // Pass distance and z to printDistance function
    }
  }
}

void printDistance(float distance, int z) {
  if (distance < 12) {
    digitalWrite(red_led, HIGH); // Turn on red LED
    digitalWrite(green_led, LOW); // Turn off green LED
    lcd.clear(); // Clear the LCD
    lcd.setCursor(0, 0);
    lcd.print("Object at");
    lcd.setCursor(0, 1);
    lcd.print(distance);
    lcd.setCursor(5, 1);
    lcd.print("in");
    lcd.setCursor(9, 1);
    lcd.print(z);
    lcd.setCursor(12, 1);
    lcd.print("deg");
  } else {
    digitalWrite(green_led, HIGH); // Turn off green LED
    digitalWrite(red_led, LOW); // Turn on red LED
    lcd.clear(); // Clear the LCD
    lcd.setCursor(0, 0);
    lcd.print("Good to go!");
  }
}
