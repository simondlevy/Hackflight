/*
   Arduino Serial1 support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#include <string.h>

extern uint8_t stream_serial1Byte; 
extern bool stream_serial1Available; 

void stream_startSerial1(void)
{
    Serial1.begin(115200);
}

void stream_serial1Write(uint8_t byte)
{
    Serial1.write(byte);
}

void stream_serial1Read(void)
{
    stream_serial1Byte = Serial1.read();
}

void stream_serial1Update(void)
{
    stream_serial1Available = Serial1.available();
}
