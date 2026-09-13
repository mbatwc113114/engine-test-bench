#include "thermocouple.h"

Thermocouple::Thermocouple() : csPin(-1), ready(false), fault(false) {}

bool Thermocouple::begin(int csPinIn) {
  csPin = csPinIn;
  if (csPin < 0) {
    ready = false;
    return false;
  }

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  ready = true;
  fault = false;
  return true;
}

bool Thermocouple::isReady() const {
  return ready;
}

float Thermocouple::readCelsius() {
  if (!ready) {
    return 0.0f;
  }
  return 0.0f;
}

bool Thermocouple::hasFault() const {
  return fault;
}
