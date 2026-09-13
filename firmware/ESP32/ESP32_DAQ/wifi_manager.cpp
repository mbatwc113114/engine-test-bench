#include "wifi_manager.h"
#include "config.h"
#include <WiFi.h>
#include <ESPmDNS.h>

bool wifiIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

String wifiIpString() {
  return WiFi.localIP().toString();
}

void wifiPrintStatus() {
  if (wifiIsConnected()) {
    Serial.print("[ESP32] WiFi connected: ");
    Serial.println(wifiIpString());
  } else {
    Serial.println("[ESP32] WiFi disconnected");
  }
}

void wifiSetup() {
#if DEBUG_MODE
  Serial.println("[ESP32] WiFi setup start");
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
#if DEBUG_MODE
    Serial.print('.');
#endif
  }

  if (wifiIsConnected()) {
    Serial.println();
    wifiPrintStatus();

    if (MDNS.begin(MDNS_NAME)) {
      Serial.println("[ESP32] mDNS started");
      MDNS.addService("engine-daq", "udp", UDP_PORT);
    } else {
      Serial.println("[ESP32] mDNS failed");
    }
  } else {
    Serial.println();
    Serial.println("[ESP32] WiFi failed to connect");
  }
}

void wifiLoop() {
  if (!wifiIsConnected()) {
    WiFi.reconnect();
  }
}
