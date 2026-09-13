#include "config.h"
#include "config_manager.h"
#include "sensor_manager.h"
#include "logger.h"
#include "protocol.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial1.setRX(7);  // Teensy RX from ESP32 TX (GPIO16)
  Serial1.setTX(8);  // Teensy TX to ESP32 RX (GPIO17)
  Serial1.begin(ESP32_UART_BAUD);
  delay(200);

  Serial.println("[TEENSY] DAQ boot");
  Serial.println("[TEENSY] UART configured for ESP32 bridge");

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
