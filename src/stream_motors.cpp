/*
   Hackflight sketch for motors

   MIT License

 */

#include "stream_motors.h"
#include <Arduino.h>

void stream_startBrushedMotors(const uint8_t * pins, uint8_t count)
{
    for (uint8_t k=0; k<count; ++k) {
        analogWriteFrequency(pins[k], 10000);
        analogWrite(pins[k], 0);
    }
}

void stream_writeBrushedMotors(const uint8_t * pins, const float * values, uint8_t count)
{
    for (uint8_t k=0; k<count; ++k) {
        analogWrite(pins[k], (uint8_t)(values[k] * 255));
    }

}
