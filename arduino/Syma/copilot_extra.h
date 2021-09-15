#pragma once

#include "copilot.h"

EXTERN float copilot_time_sec;
EXTERN bool copilot_serialAvailable;
EXTERN uint8_t copilot_serialByte;

void copilot_serialWrite(uint8_t copilot_serialWrite_arg0);
float copilot_getFloatFromSerialInput(uint8_t offset);
void copilot_collectSerialInput(uint8_t index, uint8_t value);
