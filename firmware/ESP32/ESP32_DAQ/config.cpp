#include "config.h"

DAQConfig gConfig;

void configSetup() {
  gConfig = DAQConfig();
  Serial.println("[ESP32] DAQ configuration initialized");
  Serial.print("[ESP32] UDP port: ");
  Serial.println(gConfig.udpPort);
  Serial.print("[ESP32] Servo pin: ");
  Serial.println(gConfig.servoPin);
}
