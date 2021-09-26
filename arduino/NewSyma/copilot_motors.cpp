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

static uint8_t _m1pin;
static uint8_t _m2pin;
static uint8_t _m3pin;
static uint8_t _m4pin;

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

    _m1pin = m1pin;
    _m2pin = m2pin;
    _m3pin = m3pin;
    _m4pin = m4pin;
}

void copilot_writeBrushedMotors(
        float m1value,
        float m2value,
        float m3value,
        float m4value)
{
    // printf("%3.3f %3.3f %3.3f %3.3f\n", m1value, m2value, m3value, m4value);

    writeBrushedMotor(_m1pin, m1value);
    writeBrushedMotor(_m2pin, m2value);
    writeBrushedMotor(_m3pin, m3value);
    writeBrushedMotor(_m4pin, m4value);
}
