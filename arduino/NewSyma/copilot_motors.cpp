/*
   Motor support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Servo.h>

#define _EXTERN
#include "copilot.h"

void copilot_startBrushedMotor(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

void copilot_writeBrushedMotor(uint8_t pin, float value)
{
    analogWrite(pin, (uint8_t)(value * 255));
}

static const uint8_t BRUSHLESS_MIN = 125;
static const uint8_t BRUSHLESS_MAX = 250;

void copilot_startBrushlessMotor(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    analogWrite(pin, BRUSHLESS_MIN);
}

void copilot_writeBrushlessMotor(uint8_t pin, float value)
{
    analogWrite(pin, (uint16_t)(BRUSHLESS_MIN+value*(BRUSHLESS_MAX-BRUSHLESS_MIN))); 
}

void copilot_startServoMotor(uint8_t pin)
{
    // XXX
}

void copilot_writeServoMotor(uint8_t pin, float value)
{
    // XXX
}
