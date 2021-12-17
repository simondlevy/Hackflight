/*
   Arduino debugging support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

void stream_debug(uint8_t byte)
{
    if (byte == '$') {
        printf("\n");
    }

    printf("x%02X\t", byte);
}

void stream_debug2(uint16_t value)
{
    printf("%d\n", value);
}
