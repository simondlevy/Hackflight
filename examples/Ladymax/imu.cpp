/*
Copyright (c) 2022 Simon D. Levy

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

#include <Arduino.h>
#include <Wire.h>
#include <USFSMAX.h>

#include <hackflight.h>
#include <imu.h>

// Geomagnetic field data for Lexington, Virginia on 8 June 2022.
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// Units are uT, angles are in decimal degrees.
static const float M_V             = 45.5967;
static const float M_H             = 21.5787;
static const float MAG_DECLINATION = -9.17417;

static const uint32_t I2C_CLOCK = 1000000;

// Interrupt pin
static const uint8_t INT_PIN = 10;

static const uint32_t QUAT_RATE = 500;

static USFSMAX usfsmax;

static volatile bool _gotInterrupt = true;
static void handleInterrupt()
{
    _gotInterrupt = true;
}

static float _quat[4];

static void quaternionTask(void * hackflight, uint32_t usec)
{
    (void)hackflight;
    (void)usec;

    if (usfsmax.gotQuat()) {
        usfsmax.readQuat(_quat);
    }
}

static uint32_t _gyro_interrupt_time;

uint32_t gyroInterruptTime(void)
{
    return _gyro_interrupt_time;
}

// Called in the fast inner loop
bool gyroIsReady(void)
{
    bool ready = false;

    if (_gotInterrupt) {

        usfsmax.update();

        ready = usfsmax.gotGyro();

        _gotInterrupt = false;

        _gyro_interrupt_time = ready ? micros() : _gyro_interrupt_time;
    }

    return ready;
}

int16_t gyroReadRaw(uint8_t k)
{
    return 0;
}

float gyroScale(void)
{
    return usfsmax.dpsPerCount();
}

void imuGetQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat)
{
    (void)hf;
    (void)time;

    quat->w = _quat[0];
    quat->x = _quat[1];
    quat->y = _quat[2];
    quat->z = _quat[3];
}

void imuInit(hackflight_t * hf)
{
    usfsmax.setGeoMag(M_V, M_H, MAG_DECLINATION);

    Wire.setClock(100000);      

    delay(100);

    usfsmax.begin();

    Wire.setClock(I2C_CLOCK); 

    delay(100);

    pinMode(INT_PIN, INPUT);

    attachInterrupt(INT_PIN, handleInterrupt, RISING);        

    hackflightAddSensor(hf, quaternionTask, QUAT_RATE);
}

// Unused ---------------------------------------------------------------------

int32_t imuGetGyroSkew(uint32_t nextTargetCycles, int32_t desiredPeriodCycles)
{
    return 0;
}

void imuUpdateFusion(hackflight_t * hf, uint32_t time, quaternion_t * quat, rotation_t * rot)
{
    (void)hf;
    (void)time;
    (void)quat;
    (void)rot;
}

void imuAccumulateGyro(gyro_t * gyro)
{
    (void)gyro;
}
