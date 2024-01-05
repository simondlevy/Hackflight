#pragma once

#include <stdint.h>

static const uint8_t typeLengths[] = {
    0, // none
    1, // uint8
    2, // uint6
    4, // uint32
    1, // int8
    2, // int16
    4, // int32
    4, // float
    2, // fp16
};
