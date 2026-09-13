#pragma once

#include <Arduino.h>

#define DEBUG_MODE 1
#define TEENSY_DAQ_VERSION "1.0.0"
#define TELEMETRY_HZ 60
#define SERIAL_BAUD 115200
#define ESP32_UART_BAUD 115200
#define MAX_COMMAND_LENGTH 128
#define MAX_TELEMETRY_LENGTH 256

enum DAQState {
  STATE_BOOT = 0,
  STATE_IDLE = 1,
  STATE_CONFIGURING = 2,
  STATE_READY = 3,
  STATE_RUNNING = 4,
  STATE_STOPPED = 5,
  STATE_ERROR = 6
};

struct DAQConfig {
  int lc1Data = 2;
  int lc1Clock = 3;
  int lc2Data = 4;
  int lc2Clock = 5;
  int tc1CS = 10;
  int tc2CS = 9;
  int rpmPin = 27;
  int vibrationX = 14;
  int vibrationY = 15;
  int vibrationZ = 16;
  uint32_t sampleRate = 1000;
  uint8_t rpmPulsesPerRev = 1;
};

extern DAQConfig gConfig;
extern DAQState gDaqState;
void configSetup();
bool validateConfig(const DAQConfig& cfg, String& error);
bool applyConfig(const DAQConfig& cfg, String& error);
