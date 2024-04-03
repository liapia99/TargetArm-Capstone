#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "mpu6500.h"

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display
bfs::Mpu6500 imu; // Mpu6500 object

void setup() {
  Serial.begin(9600); // Serial to display data
  while (!Serial); // Wait for Serial to initialize
  
  Wire.begin(); // Start the I2C bus
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight

  Wire.setClock(400000); // Set the I2C clock speed

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM); // Initialize and configure IMU
  if (!imu.Begin()) { // Check if IMU initialization failed
    Serial.println("Error initializing communication with IMU");
    while (1); // Infinite loop
  }

  // Set the sample rate divider
  if (!imu.ConfigSrd(19)) { // Set sample rate divider to 19
    Serial.println("Error configuring SRD");
    while (1); // Infinite loop
  }
}

void loop() {
  // Check if data read
  if (imu.Read()) {
    Serial.print("angleX : ");
    Serial.print(imu.gyro_x_radps());
    Serial.print(" angleY : ");
    Serial.print(imu.gyro_y_radps());
    Serial.print(" angleZ : ");
    Serial.println(imu.gyro_z_radps());
    Serial.println();

    // Clear previous values from the screen
    lcd.clear();

    // Display angleX
    lcdDisplay(0, 0, "X:", imu.gyro_x_radps());

    // Display angleY
    lcdDisplay(13, 0, "Y:", imu.gyro_y_radps());

    // Display angleZ
    lcdDisplay(7, 1, "Z:", imu.gyro_z_radps());

    delay(100);
  }
}

void lcdDisplay(int col, int row, const char *label, float value) {
  lcd.setCursor(col, row); // Set the cursor position
  lcd.print(label); // Print the label
  lcd.print(value, 2); // Print the value with 2 decimal places
}
