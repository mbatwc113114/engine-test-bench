#pragma once

#include <Arduino.h>

struct VibrationReadings {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

class VibrationSensor {
public:
  VibrationSensor();
  bool begin(int xPin, int yPin, int zPin);
  bool isReady() const;
  VibrationReadings read();

private:
  int xPin;
  int yPin;
  int zPin;
  bool ready;
};
