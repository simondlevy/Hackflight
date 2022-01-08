/*
   Arduino debugging support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

void debug(uint8_t byte)
{
    static uint32_t count;
    printf("%05d: x%02X\n", count++, byte);
}

void debug_rc(
        uint16_t c1,
        uint16_t c2,
        uint16_t c3,
        uint16_t c4,
        uint16_t c5,
        uint16_t c6)
{
    printf("%d %d %d %d %d %d\n", c1, c2, c3, c4, c5, c6);
}
