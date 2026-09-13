#include "teensy_link.h"
#include "config.h"
#include "udp_manager.h"
#include "nextion_manager.h"
#include <ArduinoJson.h>

bool gTeensyAlive = false;
String gTeensyRxBuffer;

static void forwardTeensyTelemetry(const TeensyTelemetryData& data) {
  StaticJsonDocument<512> doc;
  JsonObject telemetry = doc.createNestedObject("data");

  doc["type"] = "telemetry";
  doc["source"] = "teensy";
  doc["state"] = "RUNNING";
  doc["timestamp_us"] = micros();

  telemetry["rpm"] = data.rpm;
  telemetry["load_cell_1"] = data.loadCell1;
  telemetry["load_cell_2"] = data.loadCell2;
  telemetry["temp_1_c"] = data.temp1C;
  telemetry["temp_2_c"] = data.temp2C;
  telemetry["vibration_x"] = data.vibX;
  telemetry["vibration_y"] = data.vibY;
  telemetry["vibration_z"] = data.vibZ;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
  udpSendJson(jsonBuffer);

  nextionSetValue("rpm", data.rpm);
  nextionSetValue("temp1", data.temp1C);
  nextionSetValue("temp2", data.temp2C);
  nextionSetValue("vibx", data.vibX);
  nextionSetValue("viby", data.vibY);
  nextionSetValue("vibz", data.vibZ);
}

bool sendToTeensy(const String& message) {
  if (message.length() == 0) {
    return false;
  }

  Serial1.print(message);
  Serial1.print('\n');
  return true;
}

String receiveFromTeensy() {
  String value;
  while (Serial1.available()) {
    char c = static_cast<char>(Serial1.read());
    if (c == '\n' || c == '\r') {
      if (value.length() > 0) {
        return value;
      }
      continue;
    }
    value += c;
  }
  return "";
}

bool parseTeensyTelemetry(const String& msg, TeensyTelemetryData& out) {
  String payload = msg;
  payload.trim();

  if (!payload.startsWith("TLM|") && !payload.startsWith("TELEMETRY|")) {
    return false;
  }

  String body = payload.substring(payload.indexOf('|') + 1);
  int pos = 0;
  bool sawValue = false;

  while (pos >= 0 && pos < body.length()) {
    int nextSep = body.indexOf('|', pos);
    String token = (nextSep >= 0) ? body.substring(pos, nextSep) : body.substring(pos);
    int eq = token.indexOf('=');
    if (eq > 0) {
      String key = token.substring(0, eq);
      String value = token.substring(eq + 1);
      key.trim();
      value.trim();

      if (key == "rpm") {
        out.rpm = value.toFloat();
        sawValue = true;
      } else if (key == "load1") {
        out.loadCell1 = value.toFloat();
      } else if (key == "load2") {
        out.loadCell2 = value.toFloat();
      } else if (key == "temp1") {
        out.temp1C = value.toFloat();
      } else if (key == "temp2") {
        out.temp2C = value.toFloat();
      } else if (key == "vibx") {
        out.vibX = value.toFloat();
      } else if (key == "viby") {
        out.vibY = value.toFloat();
      } else if (key == "vibz") {
        out.vibZ = value.toFloat();
      }
    }

    if (nextSep < 0) {
      break;
    }
    pos = nextSep + 1;
  }

  out.timestampUs = micros();
  return sawValue;
}

bool isTeensyAlive() {
  return gTeensyAlive;
}

void teensyLinkSetup() {
  Serial1.begin(TEENSY_UART_BAUD, SERIAL_8N1, TEENSY_RX_PIN, TEENSY_TX_PIN);
  gTeensyAlive = false;
  Serial.println("[ESP32] Teensy UART ready");
  Serial.print("[ESP32] UART pins RX=");
  Serial.print(TEENSY_RX_PIN);
  Serial.print(" TX=");
  Serial.println(TEENSY_TX_PIN);
}

void teensyLinkLoop() {
  String msg = receiveFromTeensy();
  if (msg.length() == 0) {
    return;
  }

  gTeensyAlive = true;

#if DEBUG_MODE
  Serial.print("[ESP32] Teensy RX: ");
  Serial.println(msg);
#endif

  if (msg.startsWith("ACK") || msg.startsWith("ERROR") || msg.startsWith("STATUS") || msg.startsWith("PING")) {
    return;
  }

  TeensyTelemetryData data;
  if (parseTeensyTelemetry(msg, data)) {
    forwardTeensyTelemetry(data);
    setTelemetryLedState(true);
    return;
  }

  setTelemetryLedState(false);
}
