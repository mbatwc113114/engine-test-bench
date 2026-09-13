#pragma once

#include <Arduino.h>

class Thermocouple {
public:
  Thermocouple();
  bool begin(int csPin);
  bool isReady() const;
  float readCelsius();
  bool hasFault() const;

private:
  int csPin;
  bool ready;
  bool fault;
};
