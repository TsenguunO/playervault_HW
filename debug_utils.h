// debug_utils.h
#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <Arduino.h>
#include "sensor_packet.h"

namespace DebugUtils {
  inline void printPacketDebug(const SensorPacket& pkt) {
    Serial.println(F("📤 BLE IMU Packet:"));
    Serial.print  (F("  seq: "));    Serial.println(pkt.sequence);
    Serial.print  (F("  ac_x: "));   Serial.println(pkt.ac_x,   4);
    Serial.print  (F("  ac_y: "));   Serial.println(pkt.ac_y,   4);
    Serial.print  (F("  ac_z: "));   Serial.println(pkt.ac_z,   4);
    Serial.print  (F("  gyro_x: ")); Serial.println(pkt.gyro_x, 4);
    Serial.print  (F("  gyro_y: ")); Serial.println(pkt.gyro_y, 4);
    Serial.print  (F("  gyro_z: ")); Serial.println(pkt.gyro_z, 4);
    Serial.print  (F("  Lat: "));    Serial.println(pkt.lat  / 1e6, 6);
    Serial.print  (F("  Lon: "));    Serial.println(pkt.lon  / 1e6, 6);
    Serial.print  (F("  UTC: "));
    uint8_t h = (pkt.utc / 3600) + 10;  // adjust for sydney timezone
    uint8_t m = (pkt.utc % 3600) / 60;
    uint8_t s =  pkt.utc % 60;
    if (h < 24) {
      if (h < 10) Serial.print('0'); Serial.print(h);
      Serial.print(':');
      if (m < 10) Serial.print('0'); Serial.print(m);
      Serial.print(':');
      if (s < 10) Serial.print('0'); Serial.print(s);
    }
    Serial.println();
  }
}

#endif // DEBUG_UTILS_H
