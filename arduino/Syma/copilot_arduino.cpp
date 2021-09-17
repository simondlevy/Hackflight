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

void copilot_handleSerial(serial_t & serial)
{
    if (serial.count == -1) { // incomding data (e.g., set motors)

        _serialBuffer[_serialInputIndex++] = serial.input;

        switch (_serialInputIndex) {
            case 4:
                memcpy(&copilot_Input1, &_serialBuffer[0], sizeof(float));
                break;
            case 8:
                memcpy(&copilot_Input2, &_serialBuffer[4], sizeof(float));
                break;
            case 12:
                memcpy(&copilot_Input3, &_serialBuffer[8], sizeof(float));
                break;
            case 16:
                memcpy(&copilot_Input4, &_serialBuffer[12], sizeof(float));
                break;
        }
    }

    else if (serial.count > 0) { // outgoing message

        Serial.write('$');
        Serial.write('M');
        Serial.write('>');

        uint8_t size = serial.count * 4; // all values are float

        Serial.write(size);
        Serial.write(serial.type);

        uint8_t crc = size ^ serial.type;

        uint8_t * p = (uint8_t *)&serial.output01;

        for (uint8_t k=0; k<size; ++k) {
            uint8_t c = *p;
            Serial.write(c);
            crc ^= c;
            p++;
        }
        
        Serial.write(crc);
    }

    else {                      // no serial activity
        _serialInputIndex = 0;
    }
}

void copilot_convertFloat(float value)
{
    memcpy(&copilot_32bits, &value, 4);
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
    copilot_time_sec = micros() / 1.e6f;
}

// I^2C  ---------------------------------------------------------------
void copilot_startWire(void)
{
    Wire.begin();
}
