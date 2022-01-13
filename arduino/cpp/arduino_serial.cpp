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

void serial2Start(void)
{
    Serial2.begin(115200);
}

void serial2Debug(uint8_t mindex, uint16_t mvalue)
{
    Serial2.print(mindex);
    Serial2.print(": ");
    Serial2.println(mvalue);
}
