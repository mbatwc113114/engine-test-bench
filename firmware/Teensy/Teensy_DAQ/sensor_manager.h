#pragma once

#include "config.h"
#include "loadcell.h"
#include "thermocouple.h"
#include "rpm.h"
#include "vibration.h"

struct SensorStatus {
  bool loadCell1 = false;
  bool loadCell2 = false;
  bool thermocouple1 = false;
  bool thermocouple2 = false;
  bool rpm = false;
  bool vibration = false;
};

struct SensorSample {
  uint32_t timestampUs = 0;
  long loadCell1Raw = 0;
  long loadCell2Raw = 0;
  float temp1C = 0.0f;
  float temp2C = 0.0f;
  float rpm = 0.0f;
  VibrationReadings vibration;
};

class SensorManager {
public:
  SensorManager();
  bool begin(const DAQConfig& config, String& error);
  void loop();
  SensorStatus status() const;
  SensorSample sample() const;
  bool isReady() const;

private:
  LoadCell loadCell1;
  LoadCell loadCell2;
  Thermocouple thermocouple1;
  Thermocouple thermocouple2;
  RPMMonitor rpmMonitor;
  VibrationSensor vibrationSensor;
  SensorStatus sensorStatus;
  SensorSample latestSample;
  bool ready;
};

void sensorSetup();
void sensorLoop();

extern SensorManager gSensorManager;
