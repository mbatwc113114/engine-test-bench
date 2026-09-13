#pragma once

#include <Arduino.h>

void wifiSetup();
void wifiLoop();
bool wifiIsConnected();
String wifiIpString();
void wifiPrintStatus();
