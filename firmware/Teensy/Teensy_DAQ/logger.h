#pragma once

#include <Arduino.h>
#include <SD.h>

struct LogEntry {
  uint32_t timestampUs;
  float rpm;
  float loadCell1;
  float loadCell2;
  float thrust;
  float egt;
  float cht;
  float vibrationX;
  float vibrationY;
  float vibrationZ;
  float throttle;
};

class DataLogger {
public:
  DataLogger();
  bool begin();
  bool startLogging();
  bool logSample(const LogEntry& entry);
  void stopLogging();
  bool isLogging() const;
  bool hasError() const;

private:
  File logFile;
  bool logging;
  bool error;
  String fileName;
};

void loggerSetup();
void loggerLoop();
