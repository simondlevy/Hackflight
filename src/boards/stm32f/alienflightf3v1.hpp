/*
   Board class for Alienflight F3 V1

   Copyright (C) 2019 Simon D. Levy 

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

#pragma once

#include <MPU6050.h>
#include "stm32fboard.hpp"

extern "C" {

#include "drivers/light_led.h"
#include "drivers/bus_i2c.h"
#include "pg/bus_i2c.h"

#include "i2c.h"

#include "motors.hpp"

} // extern "C"

class AlienflightF3V1 : public Stm32FBoard {

    private:

        MPU6050 * _imu;

    protected: 

        bool imuReady(void)
        {
            return _imu->checkNewData();
        }

        void imuReadAccelGyro(void)
        {
            // Note reversed X/Y order because of IMU orientation
            _imu->readAccelerometer(_ay, _ax, _az);
            _imu->readGyrometer(_gy, _gx, _gz);

            // Negate for same reason
            _gx = -_gx;
            _gy = -_gy;
            _gz = -_gz;
        }

    public:

        AlienflightF3V1(void) : Stm32FBoard(usbVcpOpen())
    {
        i2c_init(I2CDEV_2);
        delaySeconds(.01);

        brushed_motors_init(0, 1, 2, 3);

        _imu = new MPU6050(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        checkImuError(_imu->begin());

        RealBoard::init();
    }

}; // class AlienflightF3V1

void Stm32FBoard::setLed(bool isOn)
{
    ledSet(0, isOn);
}
