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
    uint8_t input;
    uint8_t type;

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
    float output13;
    float output14;
    float output15;
    float output16;
    float output17;
    float output18;
    float output19;
    float output20;
    float output21;
    float output22;
    float output23;
    float output24;
    float output25;
    float output26;
    float output27;
    float output28;
    float output29;
    float output30;
    float output31;
    float output32;

} serial_t;

void copilot_serialWrite(uint8_t copilot_serialWrite_arg0);
void copilot_handleSerial(serial_t & b);
