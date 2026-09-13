#include "nextion_manager.h"

void nextionSetup() {
  Serial2.begin(115200);
  Serial.println("[ESP32] Nextion ready");
}

void nextionLoop() {
  // Nextion updates are throttled to a low refresh rate. The actual protocol
  // should be isolated here when the display is wired and configured.
}

void nextionSetValue(const String& name, float value) {
  if (name.length() == 0) {
    return;
  }

  Serial2.print(name);
  Serial2.print(".val=");
  Serial2.print(value, 1);
  Serial2.print("\xFF\xFF\xFF");
}
