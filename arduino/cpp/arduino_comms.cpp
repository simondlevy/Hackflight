/*
   Arduino support for serial communications

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <stdint.h>

void commsWrite(uint8_t byte)
{
    Serial.write(byte);
}

