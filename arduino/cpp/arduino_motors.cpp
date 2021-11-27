/*
   Arduino motor support

   MIT License

 */

#include <Arduino.h>

void startBrushedMotor(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

void writeBrushedMotor(uint8_t pin, float val)
{
    analogWrite(pin, (uint8_t)(val*255));
}

void startBrushlessMotor(uint16_t pwm_min, uint8_t pin)
{
    pinMode(pin, OUTPUT);
    analogWrite(pin, pwm_min);
}

void writeBrushlessMotor(uint16_t pwm_min, uint16_t pwm_max, uint8_t pin, float val)
{
    analogWrite(pin, (uint16_t)(pwm_min + val * (pwm_max - pwm_min))); 
}
