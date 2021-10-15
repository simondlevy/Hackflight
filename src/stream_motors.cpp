/*
   Hackflight sketch for motors

   MIT License

 */

#include "stream_motors.h"
#include <Arduino.h>

static void startBrushedMotor(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

void stream_startBrushedMotors(uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin)
{
    startBrushedMotor(m1_pin);
    startBrushedMotor(m2_pin);
    startBrushedMotor(m3_pin);
    startBrushedMotor(m4_pin);
}

void stream_writeBrushedMotors(const uint8_t * pins, const float * values, uint8_t count)
{
    for (uint8_t k=0; k<count; ++k) {
        analogWrite(pins[k], (uint8_t)(values[k] * 255));
    }

}
