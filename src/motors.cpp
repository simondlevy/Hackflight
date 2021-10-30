/*
   Platform-independent support for motor functions that can't be done in Haskell

   MIT License

 */

#include <stdint.h>

void startBrushedMotor(uint8_t pin);
void writeBrushedMotor(uint8_t pin, float val);

// This is a way to still have good gyro corrections if at
// least one motor reaches its max
static float scale(float val, float max)
{
    val = max > 1 ?  val - max + 1 : val;
    return val < 0 ? 0 : val > 1 ? 1 : val;
}

void stream_writeBrushedMotors(
        uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin,
        float m1_val, float m2_val, float m3_val, float m4_val)
{
    void writeBrushedMotor(uint8_t pin, float value);

    float max = m1_val;
    if (m2_val > m1_val) max = m2_val;
    if (m3_val > m2_val) max = m3_val;
    if (m4_val > m3_val) max = m4_val;

    m1_val = scale(m1_val, max);
    m2_val = scale(m2_val, max);
    m3_val = scale(m3_val, max);
    m4_val = scale(m4_val, max);

    writeBrushedMotor(m1_pin, m1_val);
    writeBrushedMotor(m2_pin, m2_val);
    writeBrushedMotor(m3_pin, m3_val);
    writeBrushedMotor(m4_pin, m4_val);
}

void stream_startBrushedMotors(uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin)
{
    startBrushedMotor(m1_pin);
    startBrushedMotor(m2_pin);
    startBrushedMotor(m3_pin);
    startBrushedMotor(m4_pin);
}


