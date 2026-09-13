#include "udp_manager.h"
#include "config.h"
#include "servo_manager.h"
#include "teensy_link.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

WiFiUDP gUdp;
IPAddress gGuiIp;
uint16_t gGuiPort = 0;

bool udpHasClient() {
  return gGuiPort != 0 && gGuiIp != IPAddress(0, 0, 0, 0);
}

void udpSetClient(IPAddress ip, uint16_t port) {
  gGuiIp = ip;
  gGuiPort = port;
}

void udpSetup() {
  gUdp.begin(UDP_PORT);
  Serial.print("[ESP32] UDP listening on ");
  Serial.println(UDP_PORT);
}

void udpLoop() {
  int packetSize = gUdp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  char buffer[512];
  int len = gUdp.read(buffer, sizeof(buffer) - 1);
  if (len <= 0) {
    return;
  }
  buffer[len] = '\0';

  gGuiIp = gUdp.remoteIP();
  gGuiPort = gUdp.remotePort();

#if DEBUG_MODE
  Serial.print("[ESP32] UDP RX: ");
  Serial.println(buffer);
#endif

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, buffer);
  if (err) {
    udpSendJson("{\"type\":\"ERROR\",\"error\":\"INVALID_JSON\"}");
    return;
  }

  const char* rawType = doc["type"] | doc["command"] | "";
  const char* state = doc["state"] | "";

  String command = String(rawType);
  command.trim();
  command.toUpperCase();

  String stateValue = String(state);
  stateValue.trim();
  stateValue.toUpperCase();

  const char* type = command.c_str();
  const char* stateKey = stateValue.c_str();

  if (strcmp(type, "HELLO") == 0 || strcmp(type, "PING") == 0) {
    String reply = String("{\"type\":\"ACK\",\"command\":\"") + type + "\",\"state\":\"READY\",\"ip\":\"" + WiFi.localIP().toString() + "\",\"port\":" + String(UDP_PORT) + "}";
    udpSendJson(reply.c_str());
    return;
  }

  if (strcmp(type, "STATUS") == 0 || strcmp(type, "GET_STATUS") == 0) {
    udpSendJson("{\"type\":\"STATUS\",\"state\":\"READY\",\"wifi\":\"CONNECTED\"}");
    return;
  }

  if (strcmp(type, "SET_THROTTLE") == 0 || strcmp(type, "THROTTLE") == 0) {
    float throttle = doc["value"] | doc["percent"] | 0.0f;
    setThrottlePercent(throttle);
    String reply = String("{\"type\":\"ACK\",\"command\":\"SET_THROTTLE\",\"value\":") + String(throttle, 2) + "}";
    udpSendJson(reply.c_str());
    return;
  }

  if (strcmp(type, "STOP") == 0 || strcmp(type, "DAQ_STOP") == 0) {
    setThrottlePercent(0.0f);
    sendToTeensy("STOP");
    udpSendJson("{\"type\":\"ACK\",\"command\":\"STOP\",\"state\":\"STOPPED\"}");
    return;
  }

  if (strcmp(stateKey, "START") == 0 || strcmp(type, "START") == 0 || strcmp(type, "DAQ_START") == 0) {
    sendToTeensy("DAQ_START");
    udpSendJson("{\"type\":\"ACK\",\"command\":\"DAQ_START\",\"state\":\"RUNNING\"}");
    return;
  }

  if (strcmp(type, "TELEMETRY") == 0) {
    udpSendJson("{\"type\":\"ACK\",\"command\":\"TELEMETRY\",\"state\":\"RUNNING\"}");
    return;
  }

  udpSendJson("{\"type\":\"ERROR\",\"error\":\"INVALID_COMMAND\"}");
}

void udpSendJson(const char* jsonText) {
  if (!udpHasClient()) {
    return;
  }

  gUdp.beginPacket(gGuiIp, gGuiPort);
  gUdp.print(jsonText);
  gUdp.endPacket();
}
