/*
   Hackflight sketch for motors

   MIT License

 */

#include <stdint.h>

void stream_startBrushedMotors(const uint8_t * pins, const uint8_t count=4);

void stream_writeBrushedMotors(const uint8_t * pins, float * values, const uint8_t count=4);
