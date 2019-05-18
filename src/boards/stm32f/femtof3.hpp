/*
   Board class for FemtoF3

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

#include <MPU6500.h>
#include "stm32fboard.hpp"

extern "C" {
#include "drivers/light_led.h"
#include "spi.h"
#include "motors.hpp"
}

class FemtoF3 : public Stm32FBoard {

    private:

        MPU6500 * _imu;

    protected: 

        // SoftwareQuaternionBoard class overrides

        virtual bool imuReady(void) override
        {
            return _imu->checkNewData();
        }

        virtual void imuReadAccelGyro(void) override
        {
            _imu->readAccelerometer(_ax, _ay, _az);
            _imu->readGyrometer(_gx, _gy, _gz);

            // Negate for IMU orientation
            _ay = -_ay;
            _gx = -_gx;
            _gz = -_gz;
        }


    public:

        FemtoF3(void) : Stm32FBoard(usbVcpOpen())
    {
        spi_init(MPU6500_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6500_CS_PIN)));

        brushless_motors_init(0, 1, 2, 3);

        _imu = new MPU6500(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        checkImuError(_imu->begin());

        RealBoard::init();
    }

}; // class FemtoF3

void Stm32FBoard::setLed(bool isOn)
{
    ledSet(0, isOn);
}


