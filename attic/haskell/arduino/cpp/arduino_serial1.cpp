/*
   Arduino Serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#include <string.h>

extern uint8_t serial1Byte; 
extern bool serial1Available; 

void serial1Start(void)
{
    Serial1.begin(115200);
}

void serial1Write(uint8_t byte)
{
    Serial1.write(byte);
}

void serial1Read(void)
{
    serial1Byte = Serial1.read();
}

void serial1Update(void)
{
    serial1Available = Serial1.available();
}

void serial1Debug(void)
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
