#include "safety_manager.h"
#include "servo_manager.h"
#include "config.h"

bool gSafetyActive = false;

bool safetyIsActive() {
  return gSafetyActive;
}

void safetySetup() {
  gSafetyActive = false;
  setThrottlePercent(SAFE_THROTTLE_PERCENT);
}

void safetyLoop() {
  // Safety conditions are checked here. Communication timeout or invalid state
  // should force the servo to the safe throttle and block resuming until a new
  // explicit START command is issued.
  if (gSafetyActive) {
    setThrottlePercent(SAFE_THROTTLE_PERCENT);
  }
}
