/*
   Hackflight sketch for motors

   MIT License

 */

#include <Arduino.h>


static void startBrushedMotor(uint8_t pin)
{
    analogWriteFrequency(pin, 10000);
    analogWrite(pin, 0);
}

// XXX causes code size blowup if we put it in Mixer.hs
static void normalize(float * values, uint8_t count)
{
    float maxval = 0;

    for (uint8_t k=0; k<count; ++k) {
        maxval = values[k] > maxval ?  values[k] : maxval;
    }

    for (uint8_t k=0; k<count; ++k) {

        // This is a way to still have good gyro corrections if at
        // least one motor reaches its max
        values[k] = maxval > 1 ?  values[k] - maxval + 1 : values[k];

        // Keep motor values in appropriate interval
        values[k] = values[k] < 0 ? 0 : values[k] > 1 ? 1 : values[k];
    }
}

void stream_startBrushedMotors(uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin)
{
    startBrushedMotor(m1_pin);
    startBrushedMotor(m2_pin);
    startBrushedMotor(m3_pin);
    startBrushedMotor(m4_pin);
}

void stream_writeBrushedMotors(const uint8_t * pins, float * values, uint8_t count)
{
    normalize(values, count);

    for (uint8_t k=0; k<count; ++k) {
        analogWrite(pins[k], (uint8_t)(values[k] * 255));
    }

}
