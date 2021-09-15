#include <Arduino.h>
#include "copilot_arduino.h"

#define _EXTERN
#include "copilot_extra.h"

// Serial comms ---------------------------------------------------------------

void startSerial(void)
{
    Serial.begin(115200);
}

void copilot_serialWrite(uint8_t b)
{
    Serial.write(b);
}

void updateSerial(void)
{
    copilot_serialAvailable = Serial.available();

    if (copilot_serialAvailable) {
        copilot_serialByte = Serial.read();
    }
}

static uint8_t serialInputBuffer[128] = {};

float copilot_getFloatFromSerialInput(uint8_t offset)
{
    float value = 0;
    memcpy(&value,  &serialInputBuffer[offset], sizeof(float));
    return value;
}

void copilot_collectSerialInput(uint8_t index, uint8_t value)
{
   serialInputBuffer[index] = value;
}

// LED---------------------------------------------------------------

static uint8_t _led_pin;

void startLed(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    _led_pin = pin;
}

void copilot_setLed(bool on)
{
    digitalWrite(_led_pin, on);
}

// Clock ---------------------------------------------------------------

void updateClock(void)
{
    copilot_time_sec = micros() / 1.e6f;
}
