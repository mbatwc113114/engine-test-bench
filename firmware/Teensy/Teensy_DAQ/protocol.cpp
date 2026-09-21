#include "protocol.h"
#include "config_manager.h"
#include "sensor_manager.h"

namespace {
  const size_t kMaxLineLength = 128;
  const uint32_t kTelemetryIntervalMs = 1000UL / TELEMETRY_HZ;
  uint32_t gNextId = 1;
  uint32_t gLastTelemetryMs = 0;
  // UART bytes do not necessarily arrive in one loop iteration.  Keep each
  // port's partial line so ESP32 commands are not discarded mid-command.
  String gUsbRxBuffer;
  String gEsp32RxBuffer;

  String trimString(const String& value) {
    String result = value;
    result.trim();
    return result;
  }

  void processPort(Stream& port, String& buffer, const char* portName) {
    while (port.available()) {
      const char ch = static_cast<char>(port.read());
      if (ch == '\n' || ch == '\r') {
        if (buffer.length() > 0) {
          String response;
          if (processProtocolLine(buffer, response)) {
            port.println(response);
#if DEBUG_MODE
            Serial.print("[TEENSY] ");
            Serial.print(portName);
            Serial.print(" RX: ");
            Serial.println(buffer);
#endif
          }
          buffer = "";
        }
        continue;
      }

      if (buffer.length() >= kMaxLineLength - 1) {
        buffer = "";
        port.println(buildError("PROTOCOL", "INVALID_COMMAND", gNextId++));
      } else {
        buffer += ch;
      }
    }
  }

}

String buildAck(const String& command, uint32_t id) {
  String msg = String(ACK_PREFIX) + "|id=" + String(id) + "|command=" + command + "|status=OK";
  return msg;
}

String buildError(const String& command, const String& reason, uint32_t id) {
  String msg = String(ERROR_PREFIX) + "|id=" + String(id) + "|command=" + command + "|error=" + reason;
  return msg;
}

String buildTelemetryMessage() {
  SensorSample sample = gSensorManager.sample();

  String msg = "TLM|";
  msg += "rpm=" + String(sample.rpm, 2) + "|";
  msg += "load1=" + String(static_cast<float>(sample.loadCell1Raw), 2) + "|";
  msg += "load2=" + String(static_cast<float>(sample.loadCell2Raw), 2) + "|";
  msg += "temp1=" + String(sample.temp1C, 2) + "|";
  msg += "temp2=" + String(sample.temp2C, 2) + "|";
  msg += "vibx=" + String(sample.vibration.x, 3) + "|";
  msg += "viby=" + String(sample.vibration.y, 3) + "|";
  msg += "vibz=" + String(sample.vibration.z, 3);

  return msg;
}

bool processProtocolLine(const String& line, String& response) {
  String input = trimString(line);
  if (input.length() == 0) {
    return false;
  }

  // Responses and telemetry are never commands.  Ignoring them here prevents
  // feedback loops if a line is ever reflected onto the shared UART.
  if (input.startsWith("TLM|") || input.startsWith("TELEMETRY|") ||
      input.startsWith("ACK|") || input.startsWith("ERROR|")) {
    response = "";
    return false;
  }

  if (input == PING_COMMAND) {
    response = buildAck(PING_COMMAND, gNextId++);
    return true;
  }

  if (input == STATUS_COMMAND) {
    response = buildAck(STATUS_COMMAND, gNextId++) + "|state=" + String(getDaqState());
    return true;
  }

  if (input.startsWith(String(SET_CONFIG_COMMAND) + " ")) {
    if (gDaqState == STATE_RUNNING) {
      response = buildError(SET_CONFIG_COMMAND, "CONFIG_NOT_ALLOWED_WHILE_RUNNING", gNextId++);
      return true;
    }

    String payload = input.substring(strlen(SET_CONFIG_COMMAND) + 1);
    payload.trim();
    DAQConfig cfg = gConfig;

    int values[10] = {0};
    int count = 0;
    char* ptr = strtok((char*)payload.c_str(), ",");
    while (ptr != nullptr && count < 10) {
      values[count++] = atoi(ptr);
      ptr = strtok(nullptr, ",");
    }

    if (count < 10) {
      response = buildError(SET_CONFIG_COMMAND, "INVALID_CONFIGURATION", gNextId++);
      return true;
    }

    cfg.lc1Data = values[0];
    cfg.lc1Clock = values[1];
    cfg.lc2Data = values[2];
    cfg.lc2Clock = values[3];
    cfg.tc1CS = values[4];
    cfg.tc2CS = values[5];
    cfg.rpmPin = values[6];
    cfg.vibrationX = values[7];
    cfg.vibrationY = values[8];
    cfg.vibrationZ = values[9];

    String error;
    if (!applyConfig(cfg, error)) {
      response = buildError(SET_CONFIG_COMMAND, error, gNextId++);
      return true;
    }

    response = buildAck(SET_CONFIG_COMMAND, gNextId++);
    return true;
  }

  if (input == START_COMMAND) {
    if (gDaqState == STATE_IDLE || gDaqState == STATE_READY || gDaqState == STATE_STOPPED) {
      setDaqState(STATE_RUNNING);
#if DEBUG_MODE
      Serial.println("[TEENSY] DAQ_START accepted");
      Serial.println("[TEENSY] DAQ state -> RUNNING");
#endif
      response = buildAck(START_COMMAND, gNextId++) + "|state=RUNNING";
      return true;
    }

    response = buildError(START_COMMAND, "NOT_READY", gNextId++);
    return true;
  }

  if (input == "PING_TEENSY") {
    response = buildAck("PING_TEENSY", gNextId++) + "|state=" + String(gDaqState);
    return true;
  }

  if (input == STOP_COMMAND) {
    setDaqState(STATE_STOPPED);
#if DEBUG_MODE
    Serial.println("[TEENSY] STOP accepted");
    Serial.println("[TEENSY] DAQ state -> STOPPED");
#endif
    response = buildAck(STOP_COMMAND, gNextId++) + "|state=STOPPED";
    return true;
  }

  response = buildError(input, "INVALID_COMMAND", gNextId++);
  return true;
}

void protocolSetup() {
  Serial.println("[TEENSY] Protocol ready");
}

void protocolLoop() {
  processPort(Serial, gUsbRxBuffer, "USB");
  processPort(Serial2, gEsp32RxBuffer, "ESP32");

  if (gDaqState == STATE_RUNNING) {
    uint32_t now = millis();
    if (now - gLastTelemetryMs >= kTelemetryIntervalMs) {
      gLastTelemetryMs = now;
      String telemetry = buildTelemetryMessage();
      Serial2.println(telemetry);
#if DEBUG_MODE
      Serial.print("[TEENSY] TLM TX: ");
      Serial.println(telemetry);
#endif
    }
  }
}
