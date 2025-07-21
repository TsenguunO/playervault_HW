#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <bluefruit.h>

// BLE service & characteristic UUIDs
BLEService imuService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic imuChar("abcdef01-1234-5678-1234-56789abcdef0");

// BLE connection variables
uint16_t currentMtu = 23;
bool readyToNotify = false;

// ========== Sensor Packet ==========
#pragma pack(push, 1)
typedef struct {
  int32_t sequence;
  float ac_x;
  float ac_y;
  float ac_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  int32_t lat;
  int32_t lon;
  uint32_t utc;
} SensorPacket;
#pragma pack(pop)

SensorPacket* sensorDataGlobal = nullptr;

// ========== Connection Callback ==========
void connect_callback(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  connection->requestMtuExchange(64);  // request MTU > 31
  delay(100);

  currentMtu = connection->getMtu();
  Serial.print("📏 Peripheral MTU: ");
  Serial.println(currentMtu);

  readyToNotify = true;
}

// ========== Notify Function ==========
void notifyIMUData() {
  if (!readyToNotify || !sensorDataGlobal) return;

  uint8_t* rawData = (uint8_t*)sensorDataGlobal;
  size_t totalSize = sizeof(SensorPacket);

  // Either send in one shot or chunked based on MTU
  if (currentMtu >= totalSize + 3) {
    imuChar.notify(rawData, totalSize);
  } else {
    size_t chunkSize = (currentMtu >= 23) ? currentMtu - 3 : 20;
    for (size_t i = 0; i < totalSize; i += chunkSize) {
      size_t len = min(chunkSize, totalSize - i);
      imuChar.notify(rawData + i, len);
      delay(5);
    }
  }
}

// ========== BLE Setup ==========
void setupBLE(SensorPacket* sensorDataRef) {
  sensorDataGlobal = sensorDataRef;

  Bluefruit.configPrphConn(64, 12, 2, 1);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("01_PlayerVault_T");
  Bluefruit.Periph.setConnectCallback(connect_callback);

  imuChar.setProperties(CHR_PROPS_NOTIFY);
  imuChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuChar.setMaxLen(sizeof(SensorPacket));
  imuChar.setFixedLen(false);

  imuService.begin();
  imuChar.begin();

  delay(1000);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(imuService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  delay(1000);
  Bluefruit.Advertising.start(0);
}

#endif
