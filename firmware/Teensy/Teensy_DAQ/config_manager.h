#pragma once

#include "config.h"

void configSetup();
bool validateConfig(const DAQConfig& cfg, String& error);
bool applyConfig(const DAQConfig& cfg, String& error);
const DAQConfig& getDefaultConfig();
DAQState getDaqState();
void setDaqState(DAQState state);
