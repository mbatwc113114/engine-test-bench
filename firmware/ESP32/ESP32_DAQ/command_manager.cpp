#include "command_manager.h"
#include "udp_manager.h"
#include "config.h"
#include "servo_manager.h"
#include <ArduinoJson.h>

void handleCommandString(const String& jsonMessage) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, jsonMessage);
  if (err) {
    udpSendJson("{\"type\":\"ERROR\",\"error\":\"INVALID_JSON\"}");
    return;
  }

  const char* command = doc["command"] | doc["type"] | "";
  if (strcmp(command, "SET_THROTTLE") == 0) {
    float value = doc["value"] | 0.0f;
    setThrottlePercent(value);
    udpSendJson("{\"type\":\"ACK\",\"command\":\"SET_THROTTLE\",\"value\":0}");
  }
}

void commandSetup() {
  Serial.println("[ESP32] Command manager ready");
}

void commandLoop() {
  // The actual command routing is handled by UDP receive processing. This hook is
  // kept for future command queueing and validation logic.
}
