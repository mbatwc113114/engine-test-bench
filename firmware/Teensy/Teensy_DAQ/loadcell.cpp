#include "loadcell.h"

LoadCell::LoadCell()
  : dataPin(-1), clockPin(-1), ready(false), offset(0), calibrationFactor(1.0f) {
}

bool LoadCell::begin(int dataPinIn, int clockPinIn) {
  dataPin = dataPinIn;
  clockPin = clockPinIn;

  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  digitalWrite(clockPin, LOW);

  ready = (dataPin >= 0 && clockPin >= 0);
  tare();
  return ready;
}

bool LoadCell::isReady() const {
  return ready;
}

long LoadCell::readRaw() {
  if (!ready) {
    return 0;
  }

  long value = 0;
  digitalWrite(clockPin, LOW);

  for (int i = 0; i < 24; ++i) {
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(1);
    value <<= 1;
    if (digitalRead(dataPin)) {
      value += 1;
    }
    digitalWrite(clockPin, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(clockPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(clockPin, LOW);

  value ^= 0x800000;
  return value;
}

float LoadCell::readKg() {
  if (!ready) {
    return 0.0f;
  }

  long raw = readRaw() - offset;
  return raw / calibrationFactor;
}

float LoadCell::readNewton() {
  return readKg() * 9.81f;
}

void LoadCell::tare() {
  if (!ready) {
    return;
  }

  offset = readRaw();
}

void LoadCell::setCalibrationFactor(float factor) {
  if (factor > 0.0f) {
    calibrationFactor = factor;
  }
}
