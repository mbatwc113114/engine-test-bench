#pragma once

#include "config.h"

#define PING_COMMAND "PING"
#define STATUS_COMMAND "GET_STATUS"
#define SET_CONFIG_COMMAND "SET_CONFIG"
#define START_COMMAND "DAQ_START"
#define STOP_COMMAND "STOP"
#define ACK_PREFIX "ACK"
#define ERROR_PREFIX "ERROR"

void protocolSetup();
void protocolLoop();
bool processProtocolLine(const String& line, String& response);
String buildAck(const String& command, uint32_t id = 0);
String buildError(const String& command, const String& reason, uint32_t id = 0);
