/* 
   USFS interrupt example

   Copyright (C) 2022 TleraCorp && Simon D. Levy

   Adapted from

     https://github.com/kriswiner/USFS_SENtral_sensor_hub/tree/master/WarmStartandAccelCal

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include <Wire.h>

#include "USFS.h"

// Set to 0 for polling version
static const uint8_t INTERRUPT_PIN = 12; 

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint8_t INTERRUPT_ENABLE = USFS_INTERRUPT_RESET_REQUIRED |
                                        USFS_INTERRUPT_ERROR |
                                        USFS_INTERRUPT_GYRO |
                                        USFS_INTERRUPT_QUAT;

static const bool VERBOSE = true;

static const uint8_t REPORT_HZ = 2;

static volatile bool _gotNewImuData;
static void imuInterruptHandler()
{
    _gotNewImuData = true;
}

static volatile bool _gotNewRxData;
void serialEvent1(void)
{
    _gotNewRxData = true;
}

void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(100);

    usfsLoadFirmware(); 

    usfsBegin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE,
            INTERRUPT_ENABLE);

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, imuInterruptHandler, RISING);  

    // Clear interrupts
    usfsCheckStatus();

} // setup

void loop()
{
    static uint32_t _gyroCount;
    static uint32_t _quatCount;

    if (_gotNewImuData) { 

        _gotNewImuData = false;  

        uint8_t eventStatus = usfsCheckStatus(); 

        if (usfsEventStatusIsError(eventStatus)) { 

            usfsReportError(eventStatus);
        }

        if (usfsEventStatusIsGyrometer(eventStatus)) { 

            float gx=0, gy=0, gz=0;
            usfsReadGyrometer(gx, gy, gz);
            _gyroCount++;
        }

        if (usfsEventStatusIsQuaternion(eventStatus)) { 
            float qw=0, qx=0, qy=0, qz=0;
            usfsReadQuaternion(qw, qx, qy, qz);
            _quatCount++;
        }

    } 

    static uint32_t _rxCount;

    if (_gotNewRxData) { 
        _gotNewRxData = false;  
        _rxCount++;
    }

    static uint32_t _msec;

    uint32_t msec = millis();

    if (msec-_msec > 1000/REPORT_HZ) { 

        printf("Gyro=%d  Quat=%d  Receiver=%d\n",
            (int)_gyroCount*REPORT_HZ,
            (int)_quatCount*REPORT_HZ,
            (int)_rxCount*REPORT_HZ);


        _gyroCount = 0;
        _quatCount = 0;
        _msec = msec;
    } 

}  // loop
