/*
   Hackflight stream-based serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#include <string.h>

extern uint8_t stream_serialByte; 
extern bool stream_serialAvailable; 

void stream_startSerial(void)
{
    Serial.begin(115200);
}

void stream_serialWrite(uint8_t byte)
{
    Serial.write(byte);
}

void stream_serialRead(void)
{
    stream_serialByte = Serial.read();
}

void stream_serialUpdate(void)
{
    stream_serialAvailable = Serial.available();
}
