#include "config_manager.h"

DAQConfig gConfig;
DAQState gDaqState = STATE_BOOT;

const DAQConfig& getDefaultConfig() {
  static const DAQConfig defaults;
  return defaults;
}

bool validateConfig(const DAQConfig& cfg, String& error) {
  error = "";

  if (cfg.sampleRate == 0 || cfg.sampleRate > 5000) {
    error = "INVALID_CONFIGURATION: sample rate out of range";
    return false;
  }

  if (cfg.rpmPin < 0 || cfg.rpmPin > 39) {
    error = "INVALID_CONFIGURATION: rpm pin invalid";
    return false;
  }

  int pins[] = {
    cfg.lc1Data, cfg.lc1Clock,
    cfg.lc2Data, cfg.lc2Clock,
    cfg.tc1CS, cfg.tc2CS,
    cfg.rpmPin, cfg.vibrationX, cfg.vibrationY, cfg.vibrationZ
  };

  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    if (pins[i] < 0 || pins[i] > 39) {
      error = "INVALID_CONFIGURATION: pin out of range";
      return false;
    }
  }

  for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
    for (size_t j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j) {
      if (pins[i] == pins[j] && pins[i] != -1) {
        error = "PIN_CONFLICT: duplicate pin assignment";
        return false;
      }
    }
  }

  return true;
}

bool applyConfig(const DAQConfig& cfg, String& error) {
  if (gDaqState == STATE_RUNNING) {
    error = "CONFIG_NOT_ALLOWED_WHILE_RUNNING";
    return false;
  }

  if (!validateConfig(cfg, error)) {
    return false;
  }

  gConfig = cfg;
  gDaqState = STATE_READY;
  return true;
}

DAQState getDaqState() {
  return gDaqState;
}

void setDaqState(DAQState state) {
  gDaqState = state;
}

void configSetup() {
  gConfig = getDefaultConfig();
  gDaqState = STATE_READY;

  Serial.println("[TEENSY] Config initialized");
  Serial.println("[TEENSY] Ready for DAQ_START");
  Serial.print("[TEENSY] Default sample rate: ");
  Serial.println(gConfig.sampleRate);
}
