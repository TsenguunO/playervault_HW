#include <bluefruit.h>
#include "Wire.h"
#include "LSM6DS3.h"
#include "imu_handler.h"
#include "ble_handler.h"
#include "gps_handler.h"
#include <TinyGPSPlus.h>

// ========== Instances and Globals ==========
TinyGPSPlus gps;
SensorPacket sensorPacket;
int packetCounter = 0;

const uint32_t SEND_INTERVAL = 5000;
uint32_t lastSend = 0;

void setup() {
  setupLEDs();           // Set pinMode for onboard LEDs
  setupGPS();            // Initialize GPS Serial1
  Serial.begin(115200);  // Serial monitor
  while (!Serial);

  Serial1.begin(9600);   // GPS UART
  setupIMU();            // Initialize IMU
  setupBLE(&sensorPacket);  // Pass sensorPacket to BLE
  Serial.println("✅ BLE IMU + GPS Peripheral Ready");
}

void loop() {
  double lat = 0.0;
  double lon = 0.0;
  uint32_t utc = 0;
  // 1. Feed GPS bytes continuously
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // 2. Send every second
  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    // 2a. Get IMU data
    float imuRaw[7];
    packetCounter++;
    readIMUData(imuRaw, packetCounter);

    sensorPacket.sequence = packetCounter;
    sensorPacket.ac_x = imuRaw[1];
    sensorPacket.ac_y = imuRaw[2];
    sensorPacket.ac_z = imuRaw[3];
    sensorPacket.gyro_x = imuRaw[4];
    sensorPacket.gyro_y = imuRaw[5];
    sensorPacket.gyro_z = imuRaw[6];

    // 2b. Get GPS data
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
    }
    
    if (gps.time.isValid()) {
      utc = gps.time.hour() * 3600 +
            gps.time.minute() * 60 +
            gps.time.second();
    }
    
    sensorPacket.lat = lat * 1000000;
    sensorPacket.lon = lon * 1000000;
    sensorPacket.utc = utc;
    
    // 3. Notify BLE
    if (Bluefruit.connected()) {
    imuChar.notify((uint8_t*)&sensorPacket, sizeof(sensorPacket));
    }

    // 4. Serial debug output
    Serial.println("📤 BLE IMU Packet:");
    //imu
    Serial.print("  seq: ");    Serial.println(sensorPacket.sequence);
    Serial.print("  ac_x: ");   Serial.println(sensorPacket.ac_x, 4);
    Serial.print("  ac_y: ");   Serial.println(sensorPacket.ac_y, 4);
    Serial.print("  ac_z: ");   Serial.println(sensorPacket.ac_z, 4);
    Serial.print("  gyro_x: "); Serial.println(sensorPacket.gyro_x, 4);
    Serial.print("  gyro_y: "); Serial.println(sensorPacket.gyro_y, 4);
    Serial.print("  gyro_z: "); Serial.println(sensorPacket.gyro_z, 4);
    
    //gps and time
    Serial.print("  Lat: "); Serial.println(sensorPacket.lat / 1000000.0, 6);
    Serial.print("  Lon: "); Serial.println(sensorPacket.lon / 1000000.0, 6);
    Serial.print("  UTC: ");
    uint8_t h = sensorPacket.utc / 3600;
    uint8_t m = (sensorPacket.utc % 3600) / 60;
    uint8_t s = sensorPacket.utc % 60;
    Serial.printf("%02d:%02d:%02d\n", h, m, s);
    Serial.println("--end--");
  }

  delay(5);
}
