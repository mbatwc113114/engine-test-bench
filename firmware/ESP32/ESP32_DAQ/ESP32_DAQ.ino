#include "config.h"
#include "wifi_manager.h"
#include "udp_manager.h"
#include "servo_manager.h"
#include "nextion_manager.h"
#include "command_manager.h"
#include "safety_manager.h"
#include "teensy_link.h"

namespace {
  bool gTelemetryActive = false;
  uint32_t gLastLedUpdateMs = 0;
  uint32_t gLastTelemetryMs = 0;

  void setLedDuty(uint8_t duty) {
    analogWrite(STATUS_LED_PIN, duty);
  }

  void updateStatusLed() {
    const uint32_t now = millis();

    if (gTelemetryActive && (now - gLastTelemetryMs) < 1200) {
      static float phase = 0.0f;
      phase += 0.08f;
      const float wave = (sin(phase) + 1.0f) * 0.5f;
      const uint8_t brightness = static_cast<uint8_t>(STATUS_LED_DIM_LEVEL + (wave * (255 - STATUS_LED_DIM_LEVEL)) * 0.35f);
      setLedDuty(brightness);
      gLastLedUpdateMs = now;
      return;
    }

    if (!wifiIsConnected()) {
      if (now - gLastLedUpdateMs >= 350) {
        gLastLedUpdateMs = now;
        static bool redFlash = false;
        redFlash = !redFlash;
        setLedDuty(redFlash ? STATUS_LED_DIM_LEVEL : 0);
      }
      return;
    }

    gTelemetryActive = false;
    if (now - gLastLedUpdateMs >= 1000) {
      gLastLedUpdateMs = now;
      setLedDuty(STATUS_LED_DIM_LEVEL / 2);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  analogWrite(STATUS_LED_PIN, 0);
  setLedDuty(0);

  Serial.println("ESP32 DAQ boot");

  configSetup();
  wifiSetup();
  udpSetup();
  servoSetup();
  nextionSetup();
  commandSetup();
  safetySetup();
  teensyLinkSetup();
}

void loop() {
  wifiLoop();
  udpLoop();
  commandLoop();
  servoLoop();
  nextionLoop();
  safetyLoop();
  teensyLinkLoop();

  updateStatusLed();
}

void setTelemetryLedState(bool active) {
  gTelemetryActive = active;
  if (active) {
    gLastTelemetryMs = millis();
  }
  updateStatusLed();
}
