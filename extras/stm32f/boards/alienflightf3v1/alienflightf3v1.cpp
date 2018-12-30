/*
   alienflightf3v1.cpp : Board implementation for Alienflight F3 V1

   Copyright (C) 2018 Simon D. Levy 

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "alienflightf3v1.h"


// Here we put code that interacts with Cleanflight
extern "C" {

// Cleanflight includes
#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "pg/bus_i2c.h"

// Hackflight include
#include "../../common/i2c.h"
#include "../../common/motors.h"

    AlienflightF3V1::AlienflightF3V1(void) : Stm32FBoard(usbVcpOpen())
    {
        i2c_init(I2CDEV_2);
        delaySeconds(.01);

        brushed_motors_init(0, 1, 2, 3);
        
        _imu = new MPU6050(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        checkImuError(_imu->begin());

        RealBoard::init();
    }

    void Stm32FBoard::setLed(bool isOn)
    {
        ledSet(0, isOn);
    }

    bool AlienflightF3V1::imuReady(void)
    {
        return _imu->checkNewData();
    }

    void AlienflightF3V1::imuReadAccelGyro(void)
    {
        // Note reversed X/Y order because of IMU orientation
        _imu->readAccelerometer(_ay, _ax, _az);
        _imu->readGyrometer(_gy, _gx, _gz);

        // Negate for same reason
        _gx = -_gx;
        _gy = -_gy;
        _gz = -_gz;
    }

} // extern "C"
