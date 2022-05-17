/*
   Arduino motor support

   We currently cannot call this directly from Haskell because of 
   https://github.com/Copilot-Language/copilot/issues/273

   MIT License

 */

#include <Arduino.h>

void brushedMotorStart(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

void brushedMotorWrite(uint8_t pin, float val)
{
    analogWrite(pin, (uint8_t)(val*255));
}

void brushlessMotorStart(uint16_t pwm_min, uint8_t pin)
{
    pinMode(pin, OUTPUT);
    analogWrite(pin, pwm_min);
}

void brushlessMotorWrite(uint16_t pwm_min, uint16_t pwm_max, uint8_t pin, float val)
{
    analogWrite(pin, (uint16_t)(pwm_min + val * (pwm_max - pwm_min))); 
}
