#pragma once

#include <Arduino.h>

struct TeensyTelemetryData {
  float rpm = 0.0f;
  float loadCell1 = 0.0f;
  float loadCell2 = 0.0f;
  float temp1C = 0.0f;
  float temp2C = 0.0f;
  float vibX = 0.0f;
  float vibY = 0.0f;
  float vibZ = 0.0f;
  uint32_t timestampUs = 0;
};

void teensyLinkSetup();
void teensyLinkLoop();
void setTelemetryLedState(bool active);
bool sendToTeensy(const String& message);
String receiveFromTeensy();
bool isTeensyAlive();
bool parseTeensyTelemetry(const String& msg, TeensyTelemetryData& out);
