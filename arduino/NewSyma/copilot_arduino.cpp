/*
   Arduino support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

#include "Debugger.hpp"
Debugger debugger = Debugger(&Serial2);

#define _EXTERN
#include "copilot.h"

// Serial comms ---------------------------------------------------------------

void copilot_debug(bool byte)
{
    debugger.printf("%d\n", byte);
}

void copilot_startSerial(void)
{
    Serial.begin(115200);

    debugger.begin();
}

void copilot_serialWrite(uint8_t b)
{
    Serial.write(b);
}

void copilot_updateSerial(void)
{
    copilot_serialAvailable = Serial.available();

    if (copilot_serialAvailable) {
        copilot_serialByte = Serial.read();
    }
}

static float float_from_bytes(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    float f = 0;
    uint8_t b[4] = {b0, b1, b2, b3};
    memcpy(&f, b, 4);
    return f;
}

void copilot_handleSerialJnput(
        uint8_t b00,
        uint8_t b01,
        uint8_t b02,
        uint8_t b03,
        uint8_t b04,
        uint8_t b05,
        uint8_t b06,
        uint8_t b07,
        uint8_t b08,
        uint8_t b09,
        uint8_t b10,
        uint8_t b11,
        uint8_t b12,
        uint8_t b13,
        uint8_t b14,
        uint8_t b15)
{

    // debugger.printf("%x %x %x %x\n", b00, b01, b02, b03);

    copilot_input1 = float_from_bytes(b00, b01, b02, b03);

    debugger.printf("%+3.3f\n", copilot_input1);
    
    /*
    copilot_input2 = float_from_bytes(b04, b05, b06, b07);
    copilot_input3 = float_from_bytes(b08, b09, b10, b11);
    copilot_input4 = float_from_bytes(b12, b13, b14, b15);
    */
}

void copilot_sendSerialOutput(
        uint8_t type,
        uint8_t count,
        float v01, float v02, float v03, float v04,
        float v05, float v06, float v07, float v08,
        float v09, float v10, float v11, float v12)

{
    Serial.write('$');
    Serial.write('M');
    Serial.write('>');

    uint8_t size = count * 4; // all values are float

    Serial.write(size);
    Serial.write(type);

    uint8_t crc = size ^ type;

    float vals[12] = {v01, v02, v03, v04, v05, v06, v07, v08, v09, v10, v11, v12};

    uint8_t * p = (uint8_t *)vals;

    for (uint8_t k=0; k<size; ++k) {
        uint8_t c = *p;
        Serial.write(c);
        crc ^= c;
        p++;
    }

    Serial.write(crc);
}

// LED---------------------------------------------------------------

void copilot_startLed(uint8_t pin)
{
    pinMode(pin, OUTPUT);

}

void copilot_setLed(uint8_t pin, bool on)
{
    digitalWrite(pin, on);
}

// Clock ---------------------------------------------------------------

void copilot_updateClock(void)
{
    copilot_time_msec = millis();
    // copilot_time_sec = micros() / 1.e6f;
}

// I^2C  ---------------------------------------------------------------
void copilot_startWire(void)
{
    Wire.begin();
}
