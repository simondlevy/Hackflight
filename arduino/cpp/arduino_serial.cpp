/*
   Arduino Serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <string.h>
#include <stdint.h>

extern uint8_t serialByte; 
extern bool serialAvailable; 

void serialStart(void)
{
    Serial.begin(115200);
}

void serialWrite(uint8_t byte)
{
    Serial.write(byte);
}

void serialWrite(uint8_t * buf, uint8_t size)
{
    Serial.write(buf, size);
}

void serialRead(void)
{
    serialByte = Serial.read();
}

void serialUpdate(void)
{
    serialAvailable = Serial.available();
}
