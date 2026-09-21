#include "config.h"
#include "config_manager.h"
#include "sensor_manager.h"
#include "logger.h"
#include "protocol.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.setRX(7);  // Teensy RX from ESP32 GPIO16 TX
  Serial2.setTX(8);  // Teensy TX to ESP32 GPIO17 RX
  Serial2.begin(ESP32_UART_BAUD);
  delay(200);

  Serial.println("[TEENSY] DAQ boot");
  Serial.println("[TEENSY] ESP32 UART configured");
  Serial.println("[TEENSY] RX=7 TX=8 on Serial2");

  configSetup();
  sensorSetup();
  loggerSetup();
  protocolSetup();
}

void loop() {
  sensorLoop();
  loggerLoop();
  protocolLoop();
}
