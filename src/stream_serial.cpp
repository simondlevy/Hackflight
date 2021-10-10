/*
   Hackflight stream-based serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "stream_serial.h"
#include <Arduino.h>

void stream_startSerial(void)
{
    Serial.begin(115200);
}

void stream_writeSerial(uint8_t byte)
{
    Serial.write(byte);
}
