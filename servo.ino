#include <Servo.h>

Servo servo1;  // Define servo
int receivedValue = 0;  // Variable to store received data

void setup() {
  Serial.begin(9600);
  servo1.attach(3);  // Set servo to digital pin 3 
}

void loop() {
  if (Serial.available() > 0) {
    receivedValue = Serial.read() - '0';  
    moveServo(receivedValue);  // Move the servo based on the received data
  }
}

void moveServo(int value) {
  if (value == 0) {
    
    servo1.write(0);
  } else if (value == 1) {
   
    servo1.write(60);  
  
  }
}
