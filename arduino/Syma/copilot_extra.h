#pragma once

#include "copilot.h"

EXTERN float copilot_time_sec;
EXTERN bool copilot_serialAvailable;
EXTERN uint8_t copilot_serialByte;
EXTERN uint32_t copilot_32bits;

EXTERN float copilot_Input1;
EXTERN float copilot_Input2;
EXTERN float copilot_Input3;
EXTERN float copilot_Input4;

typedef struct {

    uint8_t status; // 0=nothing; 1=incoming; 2=outgoing; 3=incoming float
    uint8_t value;

} serial_t;

void copilot_serialWrite(uint8_t copilot_serialWrite_arg0);
void copilot_handleSerialByte(serial_t & b);
void copilot_convertFloat(float value);
