#pragma once

#include "copilot.h"

EXTERN float copilot_time_sec;
EXTERN bool copilot_serialAvailable;
EXTERN uint8_t copilot_serialByte;
EXTERN uint32_t copilot_32bits;

typedef struct {

    bool avail;
    uint8_t value;

} serial_t;

void copilot_serialWrite(uint8_t copilot_serialWrite_arg0);
float copilot_getFloatFromSerialInput(uint8_t offset);
void copilot_collectSerialInput(serial_t & byteIn);
void copilot_convertFloat(float value);
