/*
   femtof3.cpp : Board implmentation for FemtoF3

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

#include "femtof3.h"

// Here we put code that interacts with Cleanflight
extern "C" {

// Cleanflight includes
#include "drivers/light_led.h"

// Hackflight includes
#include "../../common/spi.h"
#include "../../common/motors.h"

    FemtoF3::FemtoF3(void) : Stm32FBoard(usbVcpOpen())
    {
        spi_init(MPU6500_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6500_CS_PIN)));

        brushless_motors_init(0, 1, 2, 3);

        _imu = new MPU6500(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        checkImuError(_imu->begin());

        RealBoard::init();
    }

    void Stm32FBoard::setLed(bool isOn)
    {
        ledSet(0, isOn);
    }

    bool FemtoF3::imuReady(void)
    {
        return _imu->checkNewData();
    }

    void FemtoF3::imuReadAccelGyro(void)
    {
        _imu->readAccelerometer(_ax, _ay, _az);
        _imu->readGyrometer(_gx, _gy, _gz);

        // Negate for IMU orientation
        _ay = -_ay;
        _gx = -_gx;
        _gz = -_gz;
    }

} // extern "C"
