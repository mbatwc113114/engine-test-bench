#include "sensor_manager.h"

namespace {
  SensorManager* gSensorManagerInstance = nullptr;
}

SensorManager::SensorManager() : ready(false) {
  gSensorManagerInstance = this;
}

bool SensorManager::begin(const DAQConfig& config, String& error) {
  error = "";

  if (!loadCell1.begin(config.lc1Data, config.lc1Clock)) {
    error = "SENSOR_INIT_FAILED: load cell 1";
    return false;
  }

  if (!loadCell2.begin(config.lc2Data, config.lc2Clock)) {
    error = "SENSOR_INIT_FAILED: load cell 2";
    return false;
  }

  if (!thermocouple1.begin(config.tc1CS)) {
    error = "SENSOR_INIT_FAILED: thermocouple 1";
    return false;
  }

  if (!thermocouple2.begin(config.tc2CS)) {
    error = "SENSOR_INIT_FAILED: thermocouple 2";
    return false;
  }

  rpmMonitor.begin(config.rpmPin, config.rpmPulsesPerRev);
  if (!vibrationSensor.begin(config.vibrationX, config.vibrationY, config.vibrationZ)) {
    error = "SENSOR_INIT_FAILED: vibration";
    return false;
  }

  sensorStatus.loadCell1 = loadCell1.isReady();
  sensorStatus.loadCell2 = loadCell2.isReady();
  sensorStatus.thermocouple1 = thermocouple1.isReady();
  sensorStatus.thermocouple2 = thermocouple2.isReady();
  sensorStatus.rpm = rpmMonitor.isReady();
  sensorStatus.vibration = vibrationSensor.isReady();
  ready = sensorStatus.loadCell1 && sensorStatus.loadCell2 && sensorStatus.thermocouple1 && sensorStatus.thermocouple2 && sensorStatus.rpm && sensorStatus.vibration;

  return ready;
}

void SensorManager::loop() {
  latestSample.timestampUs = micros();
  latestSample.loadCell1Raw = loadCell1.readRaw();
  latestSample.loadCell2Raw = loadCell2.readRaw();
  latestSample.temp1C = thermocouple1.readCelsius();
  latestSample.temp2C = thermocouple2.readCelsius();
  latestSample.rpm = rpmMonitor.readRPM();
  latestSample.vibration = vibrationSensor.read();
}

SensorStatus SensorManager::status() const {
  return sensorStatus;
}

SensorSample SensorManager::sample() const {
  return latestSample;
}

bool SensorManager::isReady() const {
  return ready;
}

SensorManager gSensorManager;

void sensorSetup() {
  String error = "";
  if (!gSensorManager.begin(gConfig, error)) {
    Serial.print("[TEENSY] Sensor init failed: ");
    Serial.println(error);
  } else {
    Serial.println("[TEENSY] Sensors initialized");
  }
}

void sensorLoop() {
  gSensorManager.loop();
}
