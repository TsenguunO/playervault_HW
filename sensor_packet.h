// sensor_packet.h
#ifndef SENSOR_PACKET_H
#define SENSOR_PACKET_H

#include <Arduino.h>
#pragma pack(push,1)
struct SensorPacket {
  int32_t  sequence;
  float    ac_x, ac_y, ac_z;
  float    gyro_x, gyro_y, gyro_z;
  int32_t  lat, lon;
  uint32_t utc;
};
#pragma pack(pop)

#endif // SENSOR_PACKET_H
