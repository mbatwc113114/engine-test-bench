#include "vibration.h"

VibrationSensor::VibrationSensor() : xPin(-1), yPin(-1), zPin(-1), ready(false) {}

bool VibrationSensor::begin(int xPinIn, int yPinIn, int zPinIn) {
  xPin = xPinIn;
  yPin = yPinIn;
  zPin = zPinIn;

  if (xPin >= 0) {
    pinMode(xPin, INPUT);
  }
  if (yPin >= 0) {
    pinMode(yPin, INPUT);
  }
  if (zPin >= 0) {
    pinMode(zPin, INPUT);
  }

  ready = (xPin >= 0 || yPin >= 0 || zPin >= 0);
  return ready;
}

bool VibrationSensor::isReady() const {
  return ready;
}

VibrationReadings VibrationSensor::read() {
  VibrationReadings value;
  if (!ready) {
    return value;
  }

  value.x = (xPin >= 0) ? analogRead(xPin) * 0.001f : 0.0f;
  value.y = (yPin >= 0) ? analogRead(yPin) * 0.001f : 0.0f;
  value.z = (zPin >= 0) ? analogRead(zPin) * 0.001f : 0.0f;
  return value;
}
