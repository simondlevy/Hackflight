#include <Arduino.h>
#include <PulsePosition.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <i2c_t3.h>

#include "mw.h"

static void MadgwickImuUpdate( 
        float q[4], float beta, float deltat,
        float ax, float ay, float az,
        float gx, float gy, float gz) 
{
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega1, SEqDot_omega2, SEqDot_omega3, SEqDot_omega4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error

    // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * q[0];
    float halfSEq_2 = 0.5f * q[1];
    float halfSEq_3 = 0.5f * q[2];
    float halfSEq_4 = 0.5f * q[3];
    float twoSEq_1 = 2.0f * q[0];
    float twoSEq_2 = 2.0f * q[1];
    float twoSEq_3 = 2.0f * q[2];

    // Normalise the accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * q[3] - twoSEq_1 * q[2] - ax;
    f_2 = twoSEq_1 * q[1] + twoSEq_3 * q[3] - ay;
    f_3 = 1.0f - twoSEq_2 * q[1] - twoSEq_3 * q[2] - az;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * q[3];
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication

    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;

    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega1 = -halfSEq_2 * gx - halfSEq_3 * gy - halfSEq_4 * gz;
    SEqDot_omega2 = halfSEq_1 * gx + halfSEq_3 * gz - halfSEq_4 * gy;
    SEqDot_omega3 = halfSEq_1 * gy - halfSEq_2 * gz + halfSEq_4 * gx;
    SEqDot_omega4 = halfSEq_1 * gz + halfSEq_2 * gy - halfSEq_3 * gx;

    // Compute then integrate the estimated quaternion derrivative
    q[0] += (SEqDot_omega1 - (beta * SEqHatDot_1)) * deltat;
    q[1] += (SEqDot_omega2 - (beta * SEqHatDot_2)) * deltat;
    q[2] += (SEqDot_omega3 - (beta * SEqHatDot_3)) * deltat;
    q[3] += (SEqDot_omega4 - (beta * SEqHatDot_4)) * deltat;

    // Normalise quaternion
    norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}


PulsePositionInput ppm;

void board_delayMilliseconds(uint32_t msec)
{
    delay(msec);
}

uint32_t board_getMicros()
{
    return micros();
}

void board_init(void)
{
    Serial.begin(115200);

    pinMode(13, OUTPUT);  // LED

    ppm.begin(23);
}

void board_ledOff(void)
{
    digitalWriteFast(13, LOW);
}

void board_ledOn(void)
{
    digitalWriteFast(13, HIGH);
}

uint16_t board_pwmRead(uint8_t chan)
{
    return ppm.read(chan+1);
}

uint8_t board_serialAvailable(void)
{
    return Serial.available();
}

uint8_t board_serialRead(void)
{
    return Serial.read();
}

void board_serialWrite(uint8_t c)
{
    Serial.write(c);
}

void board_writeMotor(uint8_t index, uint16_t value)
{
    // XXX
}

void board_imuInit()
{
    // XXX
}

void board_imuComputeAngles()
{
    // XXX
}



void board_checkReboot(bool pendReboot)
{
}

uint16_t board_getI2cErrorCounter(void)
{
    return 0;
}

void board_reboot(void)
{
}


