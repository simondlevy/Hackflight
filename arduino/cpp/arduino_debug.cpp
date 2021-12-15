/*
   Arduino debugging support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

void stream_debug_uint8(uint8_t val)
{
    Serial.println(val);
}
