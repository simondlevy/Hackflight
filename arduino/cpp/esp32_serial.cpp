/*
   ESP32 Serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

extern uint8_t serialByte; 
extern bool serialAvailable; 

void serial1Start(uint8_t rxpin, uint8_t txpin)
{
    Serial1.begin(115200, SERIAL_8N1, rxpin, txpin);
}

void serial1Write(uint8_t byte)
{
    Serial1.write(byte);
}

void serial1Read(void)
{
    serialByte = Serial1.read();
}

void serial1Update(void)
{
    serialAvailable = Serial1.available();
}

void serial2Start(uint8_t rxpin, uint8_t txpin)
{
    Serial2.begin(115200, SERIAL_8N1, rxpin, txpin);
}

void serial2Debug(uint16_t value)
{
    Serial2.println(value);
}
