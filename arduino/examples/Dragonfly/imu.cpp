/*
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

// Dependency: https://github.com/simondlevy/mpu6050

#include <Arduino.h>

#include <MPU6050.h>

#include "gyro.h"

extern "C" {

static MPU6050 _imu;

static const uint8_t INTERRUPT_PIN = 3;

static void _isr(void)
{
}

uint32_t gyroInterruptTime(void)
{
    return 0;
}

bool gyroIsReady(void)
{
    return false;
}

int16_t gyroReadRaw(uint8_t k)
{
    return 0;
}

float gyroScale(void)
{
    return 0;
}

uint32_t gyroSyncTime(void)
{
    return 0;
}

void imuInit(void)
{
    _imu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, _isr, RISING);
}

void imuGetQuaternion(uint32_t time, bool armed, quaternion_t * quat)
{
    // XXX
}

// unused ------------------------------------------------------------------

void imuAccumulateGyro(float * gyro_curr, float * gyro_prev)
{
    (void)gyro_curr;
    (void)gyro_prev;
}

void imuUpdateFusion(timeUs_t time, quaternion_t * quat, rotation_t * rot)
{
    (void)time;
    (void)quat;
    (void)rot;

}

}
