/*
   Arduino Serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#include <string.h>

extern uint8_t stream_serial1Byte; 
extern bool stream_serial1Available; 

void stream_serial1Start(void)
{
    Serial.begin(115200);
}

void stream_serial1Write(uint8_t byte)
{
    Serial.write(byte);
}

void stream_serial1Read(void)
{
    stream_serialByte = Serial.read();
}

void stream_serial1Update(void)
{
    stream_serialAvailable = Serial.available();
}
