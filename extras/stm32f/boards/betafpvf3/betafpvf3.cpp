/*
   betafpvf3.h : Board implmentation for BetaFPV F3 Brushed board

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

#include "betafpvf3.h"

// Here we put code that interacts with Cleanflight
extern "C" {

// Hackflight includes
#include "../../common/spi.h"
#include "../../common/beeperled.h"
#include "../../common/motors.h"

    BetaFPVF3::BetaFPVF3(void) : Stm32FBoard(usbVcpOpen())
    {
        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));
        // Set up the LED (uses the beeper for some reason)
        beeperLedInit();

        brushed_motors_init(2, 3, 0, 1);

        _imu = new MPU6000(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        checkImuError(_imu->begin());

        RealBoard::init();
    }

    void Stm32FBoard::setLed(bool isOn)
    {
        beeperLedSet(isOn);
    }

    bool BetaFPVF3::imuReady(void)
    {
        return _imu->checkNewData();
    }

    void BetaFPVF3::imuReadAccelGyro(void)
    {
        _imu->readAccelerometer(_ax, _ay, _az);
        _imu->readGyrometer(_gx, _gy, _gz);

        // Negate values based on board orientation
        _az = -_az;
        _gx = -_gx;
        _gy = -_gy;
    }

} // extern "C"
