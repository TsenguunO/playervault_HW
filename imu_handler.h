#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A);

// Setup IMU
void setupIMU() {
  Wire.begin();
  if (imu.begin() != 0) {
    Serial.println("❌ IMU initialization failed!");
    while (1);
  } else {
    Serial.println("✅ IMU initialized.");
  }
}


// Read IMU sensor data into buffer
void readIMUData(float* buffer, int counter) {
  buffer[0] = counter;
  buffer[1] = imu.readFloatAccelX();
  buffer[2] = imu.readFloatAccelY();
  buffer[3] = imu.readFloatAccelZ();
  buffer[4] = imu.readFloatGyroX();
  buffer[5] = imu.readFloatGyroY();
  buffer[6] = imu.readFloatGyroZ();
}

// Setup onboard LEDs
void setupLEDs() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);    // Off
  digitalWrite(LED_GREEN, HIGH);  // Off
  digitalWrite(LED_BLUE, HIGH);   // Off
}

#endif
