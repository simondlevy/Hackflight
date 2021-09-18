#pragma once

#include "copilot.h"

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
