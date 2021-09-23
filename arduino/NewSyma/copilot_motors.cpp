/*
   Motor support for Haskell Copilot

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Servo.h>

#define _EXTERN
#include "copilot.h"

static void startBrushedMotor(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

static void writeBrushedMotor(uint8_t pin, float value)
{
    analogWrite(pin, (uint8_t)(value * 255));
}

void copilot_startBrushedMotors(
        uint8_t m1pin,
        uint8_t m2pin,
        uint8_t m3pin,
        uint8_t m4pin)
{
    startBrushedMotor(m1pin);
    startBrushedMotor(m2pin);
    startBrushedMotor(m3pin);
    startBrushedMotor(m4pin);
}

void copilot_writeBrushedMotors(
        uint8_t m1pin, float m1value,
        uint8_t m2pin, float m2value,
        uint8_t m3pin, float m3value,
        uint8_t m4pin, float m4value)
{
    writeBrushedMotor(m1pin, m1value);
    writeBrushedMotor(m2pin, m2value);
    writeBrushedMotor(m3pin, m3value);
    writeBrushedMotor(m4pin, m4value);
}
