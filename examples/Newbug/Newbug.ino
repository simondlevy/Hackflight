/* 
   USFS interrupt example

   Copyright (C) 2024 Simon D. Levy

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

#include <usfs.hpp>

#include <clock.hpp>
#include <kalman.hpp>

// Set to 0 for polling version
static const uint8_t INTERRUPT_PIN = 12; 

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
                                        Usfs::INTERRUPT_ERROR |
                                        Usfs::INTERRUPT_QUAT;

static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

static const bool VERBOSE = true;

static const uint8_t REPORT_HZ = 20;

static volatile bool _gotNewData;

static void interruptHandler()
{
    _gotNewData = true;
}

static Usfs usfs;

static KalmanFilter _kalmanFilter;

void setup()
{
    Serial.begin(115200);
    delay(4000);

    Wire.begin(); 
    Wire.setClock(400000); 
    delay(1000);

    usfs.reportChipId();        

    usfs.loadFirmware(VERBOSE); 

    usfs.begin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE,
            INTERRUPT_ENABLE,
            VERBOSE); 

    if (INTERRUPT_PIN) {
        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(INTERRUPT_PIN, interruptHandler, RISING);  
    }

    // Clear interrupts
    Usfs::checkStatus();

    _kalmanFilter.setDefaultParams();

    _kalmanFilter.init(millis());

}

void loop()
{
    static Axis3f _accel;
    static Axis3f _gyro;

    if ((INTERRUPT_PIN == 0) || _gotNewData) { 

        _gotNewData = false;  

        uint8_t eventStatus = Usfs::checkStatus(); 

        if (Usfs::eventStatusIsError(eventStatus)) { 
            Usfs::reportError(eventStatus);
        }

        if (Usfs::eventStatusIsAccelerometer(eventStatus)) { 
            usfs.readAccelerometerScaled(_accel.x, _accel.y, _accel.z);
        }

        if (Usfs::eventStatusIsGyrometer(eventStatus)) { 
            usfs.readGyrometerScaled(_gyro.x, _gyro.y, _gyro.z);
        }
    } 

    static uint32_t _nextPredictionMs;

    auto msec = millis();

    if (msec >= _nextPredictionMs) {

        _kalmanFilter.predict(msec, true); 

        _nextPredictionMs = msec + PREDICTION_UPDATE_INTERVAL_MS;
    }

    _kalmanFilter.addProcessNoise(msec);

    _kalmanFilter.updateWithAccel(_accel);

    _kalmanFilter.updateWithGyro(_gyro);

    _kalmanFilter.finalize();
}
