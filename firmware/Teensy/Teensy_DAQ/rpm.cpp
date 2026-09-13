#include "rpm.h"

namespace {
  RPMMonitor* gRPMInstance = nullptr;
}

void rpmInterruptHandler() {
  if (gRPMInstance != nullptr) {
    gRPMInstance->handleInterrupt();
  }
}

RPMMonitor::RPMMonitor()
  : pin(-1), pulsesPerRev(1), pulseCount(0), lastPulseMicros(0), lastReportMicros(0), ready(false) {
  gRPMInstance = this;
}

void RPMMonitor::begin(int pinIn, uint8_t pulsesPerRevIn) {
  pin = pinIn;
  pulsesPerRev = pulsesPerRevIn;
  pulseCount = 0;
  lastPulseMicros = micros();
  lastReportMicros = micros();

  if (pin >= 0) {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin), rpmInterruptHandler, RISING);
    ready = true;
  } else {
    ready = false;
  }
}

void RPMMonitor::reset() {
  pulseCount = 0;
  lastPulseMicros = micros();
}

void RPMMonitor::handleInterrupt() {
  uint32_t now = micros();
  pulseCount++;
  lastPulseMicros = now;
}

float RPMMonitor::readRPM() {
  if (!ready) {
    return 0.0f;
  }

  return 0.0f;
}

bool RPMMonitor::isReady() const {
  return ready;
}
