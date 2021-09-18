/*
   Arduino support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

#define _EXTERN
#include "copilot_extra.h"

// Serial comms ---------------------------------------------------------------

void copilot_startSerial(void)
{
    Serial.begin(115200);
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

static uint8_t _serialBuffer[128] = {};
static uint8_t _serialInputIndex;

void copilot_handleSerialJnput(uint8_t byte)
{
        _serialBuffer[_serialInputIndex++] = byte;

        switch (_serialInputIndex) {
            case 4:
                memcpy(&copilot_input1, &_serialBuffer[0], sizeof(float));
                break;
            case 8:
                memcpy(&copilot_input2, &_serialBuffer[4], sizeof(float));
                break;
            case 12:
                memcpy(&copilot_input3, &_serialBuffer[8], sizeof(float));
                break;
            case 16:
                memcpy(&copilot_input4, &_serialBuffer[12], sizeof(float));
                break;
        }
}

void copilot_resetSerial(void)
{
    _serialInputIndex = 0;
}

void copilot_sendSerialOutput(
        uint8_t type,
        uint8_t count,
        float v01,
        float v02,
        float v03,
        float v04,
        float v05,
        float v06,
        float v07,
        float v08,
        float v09,
        float v10,
        float v11,
        float v12)

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
