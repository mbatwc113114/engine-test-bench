#pragma once

#include <Arduino.h>

class RPMMonitor {
public:
  RPMMonitor();
  void begin(int pin, uint8_t pulsesPerRev = 1);
  void reset();
  void handleInterrupt();
  float readRPM();
  bool isReady() const;

private:
  int pin;
  uint8_t pulsesPerRev;
  volatile uint32_t pulseCount;
  volatile uint32_t lastPulseMicros;
  uint32_t lastReportMicros;
  bool ready;
};
