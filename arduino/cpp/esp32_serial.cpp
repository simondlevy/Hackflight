/*
   ESP32 Serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

extern uint8_t stream_serialByte; 
extern bool stream_serialAvailable; 

void stream_serial1Start(uint8_t rxpin, uint8_t txpin)
{
    Serial1.begin(115200, SERIAL_8N1, rxpin, txpin);
}

void stream_serial1Write(uint8_t byte)
{
    Serial1.write(byte);
}

void stream_serial1Read(void)
{
    stream_serialByte = Serial1.read();
}

void stream_serial1Update(void)
{
    stream_serialAvailable = Serial1.available();
}
