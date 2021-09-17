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

    int8_t count; // 0=nothing; -1=incoming; +=outgoing
    uint8_t type;

    uint8_t input;

    float output01;
    float output02;
    float output03;
    float output04;
    float output05;
    float output06;
    float output07;
    float output08;
    float output09;
    float output10;
    float output11;
    float output12;

} serial_t;

void copilot_serialWrite(uint8_t copilot_serialWrite_arg0);

void copilot_sendSerialOutput(
        uint8_t type,
        uint8_t count,
        float output01,
        float output02,
        float output03,
        float output04,
        float output05,
        float output06,
        float output07,
        float output08,
        float output09,
        float output10,
        float output11,
        float output12);

void copilot_resetSerial(void);

void copilot_handleSerialInput(uint8_t byte);
