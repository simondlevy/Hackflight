/*
   Arduino debugging support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

void stream_debug(uint16_t value)
{
    printf("x%04x\n", value);
}

void stream_debug2(uint8_t index, uint8_t byte)
{
    printf("%d  x%02X\n", index, byte);
}
