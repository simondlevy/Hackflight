/*
   Arduino debugging support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

void stream_debug(bool checked, uint8_t msgtype, uint8_t index, uint8_t byte)
{
    printf("checked: %d | msgtype: %d | index: %d | byte: x%02X\n", 
            checked, msgtype, index, byte);

    if (checked) {
        printf("\n");
    }
}
