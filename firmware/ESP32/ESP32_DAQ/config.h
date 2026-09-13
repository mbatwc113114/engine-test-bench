#pragma once

#include <Arduino.h>

#define DEBUG_MODE 1
#define ESP32_DAQ_VERSION "1.0.0"
#define UDP_PORT 4210
#define WIFI_SSID "GSV_R107"
#define WIFI_PASSWORD "gsv#2025"
#define MDNS_NAME "engine-daq"
#define THROTTLE_SERVO_PIN 9
#define STATUS_LED_PIN 2
#define STATUS_LED_DIM_PERCENT 20
#define STATUS_LED_DIM_LEVEL 51
#define TEENSY_UART_BAUD 115200
#define TEENSY_RX_PIN 17   // ESP32-C6 RX to Teensy TX
#define TEENSY_TX_PIN 16   // ESP32-C6 TX to Teensy RX
#define TELEMETRY_HZ 60
#define GUI_TIMEOUT_MS 3000
#define TEENSY_TIMEOUT_MS 2000
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SAFE_THROTTLE_PERCENT 0.0f

struct DAQConfig {
  int loadCell1Data = 34;
  int loadCell1Clock = 35;
  int loadCell2Data = 32;
  int loadCell2Clock = 33;
  int thermocouple1CS = 5;
  int thermocouple1SCK = 18;
  int thermocouple1MISO = 19;
  int thermocouple1MOSI = 23;
  int thermocouple2CS = 17;
  int thermocouple2SCK = 18;
  int thermocouple2MISO = 19;
  int thermocouple2MOSI = 23;
  int rpmPin = 27;
  int sampleRate = 1000;
  int udpPort = UDP_PORT;
  int servoPin = THROTTLE_SERVO_PIN;
  int servoMinUs = SERVO_MIN_US;
  int servoMaxUs = SERVO_MAX_US;
};

extern DAQConfig gConfig;
void configSetup();
