#include <bluefruit.h>
#include "Wire.h"
#include "LSM6DS3.h"
#include "imu_handler.h"
#include "ble_handler.h"
#include "gps_handler.h"
#include "debug_utils.h"
#include "sensor_packet.h"
#include <TinyGPSPlus.h>

// ========== Instances and Globals ==========
TinyGPSPlus gps;
SensorPacket sensorPacket;
int packetCounter = 0;

const uint32_t SEND_INTERVAL = 5000;
uint32_t lastSend = 0;

//void printPacketDebug(const SensorPacket& pkt);

//Flash light to inform ble status
void onConnect(uint16_t conn_handle) {
  //Green on once connected
  digitalWrite(LED_BLUE,  HIGH);
  digitalWrite(LED_GREEN, LOW);
}
void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  //Green off once disconnected
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE,  LOW);
}




void setup() {
  setupLEDs();           // Set pinMode for onboard LEDs
  digitalWrite(LED_RED, LOW); //Device is on
  
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
      digitalWrite(LED_GREEN, HIGH);
      imuChar.notify((uint8_t*)&sensorPacket, sizeof(sensorPacket));
      delay(100);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN,LOW);
      
    }
    else{
      digitalWrite(LED_GREEN, HIGH);
      }

    // 4. Serial debug output
    DebugUtils::printPacketDebug(sensorPacket);
  }

  delay(5);
}
