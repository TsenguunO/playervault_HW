#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

// Declare the GPS object (defined in your main file)
extern TinyGPSPlus gps;

// Initializes Serial1 for GPS communication
inline void setupGPS() {
  Serial1.begin(9600);  // NEO-M7 default baud rate

}

inline void readGPS() {
  while (Serial1.available()) {
    char c = Serial1.read();
    gps.encode(c); // decode, but no Serial.write(c)
  }
}

// Returns true if GPS has a valid and updated location fix
inline bool hasGPSFix() {
  return gps.location.isValid() && gps.location.isUpdated();
}

// Returns latitude (or 0.0 if no valid fix)
inline double getLatitude() {
  return gps.location.isValid() ? gps.location.lat() : 0.0;
}

// Returns longitude (or 0.0 if no valid fix)
inline double getLongitude() {
  return gps.location.isValid() ? gps.location.lng() : 0.0;
}

// Returns formatted HH:MM:SS string from GPS time
inline String getGPSTime() {
  if (gps.time.isValid()) {
    char buf[10];
    sprintf(buf, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    return String(buf);
  }
  return "NoTime";
}

#endif  // GPS_HANDLER_H
