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
