/*
   Platform-independent support for motor functions that can't be done in Haskell

   MIT License

 */

#include <stdint.h>

void startBrushedMotor(uint8_t pin);
void writeBrushedMotor(uint8_t pin, float val);
void startBrushlessMotor(uint16_t pwm_min, uint8_t pin);
void writeBrushlessMotor(uint16_t pwm_min, uint16_t pwm_max,  uint8_t pin, float val);

// This is a way to still have good gyro corrections if at
// least one motor reaches its max
static float scale(float val, float max)
{
    val = max > 1 ?  val - max + 1 : val;
    return val < 0 ? 0 : val > 1 ? 1 : val;
}

void stream_writeBrushedMotors(
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin,
        float m1_val,
        float m2_val,
        float m3_val,
        float m4_val)
{
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

void stream_startBrushedMotors(
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin)
{
    startBrushedMotor(m1_pin);
    startBrushedMotor(m2_pin);
    startBrushedMotor(m3_pin);
    startBrushedMotor(m4_pin);
}

void stream_writeBrushlessMotors(
        uint16_t pwm_min,
        uint16_t pwm_max,
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin,
        float m1_val,
        float m2_val,
        float m3_val,
        float m4_val)
{
    float max = m1_val;
    if (m2_val > m1_val) max = m2_val;
    if (m3_val > m2_val) max = m3_val;
    if (m4_val > m3_val) max = m4_val;

    m1_val = scale(m1_val, max);
    m2_val = scale(m2_val, max);
    m3_val = scale(m3_val, max);
    m4_val = scale(m4_val, max);

    writeBrushlessMotor(pwm_min, pwm_max, m1_pin, m1_val);
    writeBrushlessMotor(pwm_min, pwm_max, m2_pin, m2_val);
    writeBrushlessMotor(pwm_min, pwm_max, m3_pin, m3_val);
    writeBrushlessMotor(pwm_min, pwm_max, m4_pin, m4_val);
}

void stream_startBrushlessMotors(
        uint16_t pwm_min,
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin)
{
    startBrushlessMotor(pwm_min, m1_pin);
    startBrushlessMotor(pwm_min, m2_pin);
    startBrushlessMotor(pwm_min, m3_pin);
    startBrushlessMotor(pwm_min, m4_pin);
}
