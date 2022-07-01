/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY
   {
   }
   without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Wire.h>
#include <USFS.h>

#include <imu.h>

// Set to 0 for polling version
static const uint8_t INTERRUPT_PIN = 0 /*12*/; 

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint16_t ACCEL_SCALE = 8;
static const uint16_t GYRO_SCALE  = 2000;
static const uint16_t MAG_SCALE   = 1000;

static const uint8_t INTERRUPT_ENABLE = USFS_INTERRUPT_RESET_REQUIRED |
                                        USFS_INTERRUPT_ERROR |
                                        USFS_INTERRUPT_ACCEL |
                                        USFS_INTERRUPT_GYRO;

static const uint8_t REPORT_HZ = 2;

static bool _accelIsReady;
static int16_t _accelAdc[3];
static int16_t _gyroAdc[3];
static volatile bool _gotNewData;

static volatile uint32_t _gyroInterruptCount;
static volatile uint32_t _gyroSyncTime;


static void interruptHandler()
{
    _gotNewData = true;
    _gyroInterruptCount++;
    _gyroSyncTime = micros();
}

extern "C" {

    bool accelIsReady(void)
    {
        return _accelIsReady;
    }

    float accelRead(uint8_t axis) 
    {
        return (float)_accelAdc[axis];
    }

    uint32_t gyroInterruptCount(void)
    {
        return _gyroInterruptCount;
    }

    bool gyroIsReady(void)
    {
        bool result = false;

        if (_gotNewData) { 

            _gotNewData = false;  

            uint8_t eventStatus = usfsCheckStatus(); 

            if (usfsEventStatusIsError(eventStatus)) { 
                usfsReportError(eventStatus);
            }

            _accelIsReady = usfsEventStatusIsAccelerometer(eventStatus);

            if (_accelIsReady) {
                usfsReadAccelerometer(_accelAdc);
            }

            if (usfsEventStatusIsGyrometer(eventStatus)) { 
                usfsReadGyrometer(_gyroAdc);
                result = true;
            }

        } 

        return result;
    }

    int16_t gyroReadRaw(uint8_t k)
    {
        return _gyroAdc[k];
    }

    float gyroScale(void)
    {
        return 0.153;
    }

    uint32_t gyroSyncTime(void)
    {
        return _gyroSyncTime;
    }

    void imuInit(hackflight_t * hf, uint8_t interruptPin)
    {
        (void)hf;

        Wire.setClock(400000); 
        delay(1000);

        usfsReportChipId();        

        usfsLoadFirmware(); 

        usfsBegin(
                ACCEL_BANDWIDTH,
                GYRO_BANDWIDTH,
                ACCEL_SCALE,
                GYRO_SCALE,
                MAG_SCALE,
                QUAT_DIVISOR,
                MAG_RATE,
                ACCEL_RATE,
                GYRO_RATE,
                BARO_RATE,
                INTERRUPT_ENABLE,
                true); 

        pinMode(interruptPin, INPUT);
        attachInterrupt(interruptPin, interruptHandler, RISING);  

        // Clear interrupts
        usfsCheckStatus();
    }

} // extern "C"
