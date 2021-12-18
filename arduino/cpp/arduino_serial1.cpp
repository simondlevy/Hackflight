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

void stream_serial1Debug(void)
{
    static bool running;

    if (running) {
        printf("x%02X\n", Serial1.read());
    }
    else {
        Serial1.begin(115200);
        running = true;
    }
}
