#pragma once

#include <Arduino.h>

class LoadCell {
public:
  LoadCell();
  bool begin(int dataPin, int clockPin);
  bool isReady() const;
  long readRaw();
  float readKg();
  float readNewton();
  void tare();
  void setCalibrationFactor(float factor);

private:
  int dataPin;
  int clockPin;
  bool ready;
  long offset;
  float calibrationFactor;
};
