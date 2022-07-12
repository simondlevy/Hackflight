/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY without even the implied warranty of MERCHANTABILITY or FITNESS
   FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
   details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>

#include <USFS.h>

#include <datatypes.h>
#include <imu.h>

static const uint8_t  ACCEL_RATE_TENTH = 20; // 1/10th actual rate
static const uint8_t  GYRO_RATE_TENTH = 100; // 1/10th actual rate

// Arbitrary; unused
static const uint8_t  ACCEL_BANDWIDTH  = 3;
static const uint8_t  GYRO_BANDWIDTH   = 3;
static const uint8_t  QUAT_DIVISOR     = 1;
static const uint8_t  MAG_RATE         = 100;
static const uint8_t  BARO_RATE        = 50;

static const uint8_t INTERRUPT_ENABLE = USFS_INTERRUPT_RESET_REQUIRED |
USFS_INTERRUPT_ERROR |
USFS_INTERRUPT_GYRO |
USFS_INTERRUPT_QUAT;

static const uint8_t REPORT_HZ = 2;

static bool _accelIsReady;
static int16_t _gyroRaw[3];
static float _qw, _qx, _qy, _qz;

static volatile bool     _gotNewData;
static volatile uint32_t _gyroInterruptCount;
static volatile uint32_t _gyroSyncTime;

static void interruptHandler()
{
    _gotNewData = true;
    _gyroInterruptCount++;
    _gyroSyncTime = micros();
}

extern "C" {

    uint32_t gyroInterruptCount(void)
    {
        return _gyroInterruptCount;
    }

    bool gyroIsReady(void)
    {
        bool result = false;

        _accelIsReady = false;

        if (_gotNewData) { 

            _gotNewData = false;  

            uint8_t eventStatus = usfsCheckStatus(); 

            if (usfsEventStatusIsError(eventStatus)) { 
                usfsReportError(eventStatus);
            }

            if (usfsEventStatusIsGyrometer(eventStatus)) { 
                usfsReadGyrometerRaw(_gyroRaw);
                result = true;
            }

            if (usfsEventStatusIsQuaternion(eventStatus)) { 
                usfsReadQuaternion(_qw, _qx, _qy, _qz);
            } 

        }

        return result;
    }

    uint32_t gyroSyncTime(void)
    {
        return _gyroSyncTime;
    }

    void getQuaternion(hackflight_t * hf, uint32_t time, quaternion_t * quat)
    {
        (void)hf;
        (void)time;

        quat->w = _qw;
        quat->x = _qx;
        quat->y = _qy;
        quat->z = _qz;
    }

    int16_t gyroReadRaw(uint8_t k)
    {
        return _gyroRaw[k];
    }

    float gyroScaleDps(void)
    {
        return USFS_GYRO_SCALE;
    }

    void imuInit(hackflight_t * hf, uint8_t interruptPin)
    {
        (void)hf;

        Wire.setClock(400000); 
        delay(100);

        usfsLoadFirmware(); 

        usfsBegin(
                ACCEL_BANDWIDTH,
                GYRO_BANDWIDTH,
                QUAT_DIVISOR,
                MAG_RATE,
                ACCEL_RATE_TENTH,
                GYRO_RATE_TENTH,
                BARO_RATE,
                INTERRUPT_ENABLE);

        pinMode(interruptPin, INPUT);
        attachInterrupt(interruptPin, interruptHandler, RISING);  

        // Clear interrupts
        usfsCheckStatus();
    }

    void imuUpdate(
            hackflight_t * hf,
            uint32_t time,
            quaternion_t * quat,
            rotation_t * rot)
    {
        (void)hf;
        (void)time;
        (void)quat;
        (void)rot;
    }

    uint32_t CORE_RATE(void)
    {
        return 1000;
    }

} // extern "C"
