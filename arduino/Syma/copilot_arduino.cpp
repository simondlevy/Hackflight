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

static uint8_t _serialInputBuffer[128] = {};
static uint8_t _serialInputIndex;

void copilot_handleSerialByte(serial_t & serialByte)
{
   if (serialByte.status == 1) { // incomding data (e.g., set motors)

      _serialInputBuffer[_serialInputIndex++] = serialByte.value;

      switch (_serialInputIndex) {
          case 4:
              memcpy(&copilot_Input1, &_serialInputBuffer[0], sizeof(float));
              break;
          case 8:
              memcpy(&copilot_Input2, &_serialInputBuffer[4], sizeof(float));
              break;
          case 12:
              memcpy(&copilot_Input3, &_serialInputBuffer[8], sizeof(float));
              break;
          case 16:
              memcpy(&copilot_Input4, &_serialInputBuffer[12], sizeof(float));
              break;
      }
   }
   else {
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
