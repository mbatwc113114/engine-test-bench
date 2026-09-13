#pragma once

#include <Arduino.h>

void commandSetup();
void commandLoop();
void handleCommandString(const String& jsonMessage);
