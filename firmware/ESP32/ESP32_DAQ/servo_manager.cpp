#include "servo_manager.h"
#include "config.h"
#include <ESP32Servo.h>

Servo gThrottleServo;
float gCurrentThrottlePercent = SAFE_THROTTLE_PERCENT;

void servoSetup() {
  gThrottleServo.setPeriodHertz(50);
  gThrottleServo.attach(THROTTLE_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setThrottlePercent(SAFE_THROTTLE_PERCENT);
}

void servoLoop() {
  // Servo output is handled by explicit command calls. This keeps the actuator
  // logic non-blocking and deterministic.
}

float currentThrottlePercent() {
  return gCurrentThrottlePercent;
}

void setThrottlePercent(float percent) {
  if (!isnan(percent) && !isinf(percent)) {
    gCurrentThrottlePercent = constrain(percent, 0.0f, 100.0f);
  } else {
    gCurrentThrottlePercent = SAFE_THROTTLE_PERCENT;
  }

  int pulse = map(static_cast<int>(gCurrentThrottlePercent * 10.0f), 0, 1000, SERVO_MIN_US, SERVO_MAX_US);
  gThrottleServo.writeMicroseconds(constrain(pulse, SERVO_MIN_US, SERVO_MAX_US));
}
