/*
   Arduino motor support

   MIT License

 */

#include <Arduino.h>

static const uint16_t BRUSHLESS_MIN = 125;
static const uint16_t BRUSHLESS_MAX = 250;

void startBrushedMotor(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

void writeBrushedMotor(uint8_t pin, float val)
{
    analogWrite(pin, (uint8_t)(val*255));
}

void startBrushlessMotor(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    analogWrite(pin, BRUSHLESS_MIN);
}

void writeBrushlessMotor(uint8_t pin, float val)
{
    analogWrite(pin, (uint16_t)(BRUSHLESS_MIN + val * (BRUSHLESS_MAX - BRUSHLESS_MIN))); 
}
