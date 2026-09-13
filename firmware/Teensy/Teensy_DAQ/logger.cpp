#include "logger.h"

DataLogger::DataLogger() : logging(false), error(false) {
}

bool DataLogger::begin() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    error = true;
    Serial.println("[TEENSY] SD init failed");
    return false;
  }

  error = false;
  return true;
}

bool DataLogger::startLogging() {
  if (!begin()) {
    return false;
  }

  char filename[32];
  snprintf(filename, sizeof(filename), "EXP_%lu.CSV", millis());
  fileName = String(filename);
  logFile = SD.open(fileName.c_str(), FILE_WRITE);

  if (!logFile) {
    error = true;
    return false;
  }

  logFile.println("timestamp_us,rpm,loadcell1,loadcell2,thrust,egt,cht,vibration_x,vibration_y,vibration_z,throttle");
  logging = true;
  error = false;
  return true;
}

bool DataLogger::logSample(const LogEntry& entry) {
  if (!logging || !logFile) {
    return false;
  }

  logFile.print(entry.timestampUs);
  logFile.print(',');
  logFile.print(entry.rpm, 3);
  logFile.print(',');
  logFile.print(entry.loadCell1, 3);
  logFile.print(',');
  logFile.print(entry.loadCell2, 3);
  logFile.print(',');
  logFile.print(entry.thrust, 3);
  logFile.print(',');
  logFile.print(entry.egt, 3);
  logFile.print(',');
  logFile.print(entry.cht, 3);
  logFile.print(',');
  logFile.print(entry.vibrationX, 3);
  logFile.print(',');
  logFile.print(entry.vibrationY, 3);
  logFile.print(',');
  logFile.print(entry.vibrationZ, 3);
  logFile.print(',');
  logFile.println(entry.throttle, 3);
  return true;
}

void DataLogger::stopLogging() {
  if (logFile) {
    logFile.close();
  }
  logging = false;
}

bool DataLogger::isLogging() const {
  return logging;
}

bool DataLogger::hasError() const {
  return error;
}

DataLogger gDataLogger;

void loggerSetup() {
  gDataLogger.begin();
}

void loggerLoop() {
}
