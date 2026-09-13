#pragma once

#include <Arduino.h>

void udpSetup();
void udpLoop();
void udpSendJson(const char* jsonText);
bool udpHasClient();
void udpSetClient(IPAddress ip, uint16_t port);
