#pragma once

#include <Arduino.h>

void servoSetup();
void servoLoop();
void setThrottlePercent(float percent);
float currentThrottlePercent();
