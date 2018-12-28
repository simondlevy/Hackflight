/*
   revo.cpp : Board implementation for OpenPilot CC3D Revolution 

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

#include "revo.h"

// Here we put code that interacts with Cleanflight
extern "C" {

#include "platform.h"

    // Cleanflight includes
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"
#include "io/serial.h"
#include "target.h"
#include "stm32f4xx.h"

    // Hackflight includes
#include "../../common/spi.h"

    Revo::Revo(void) : Stm32FBoard(usbVcpOpen())
    {
        initImu();

        RealBoard::init();
    }

    void Revo::initImu(void)
    {
        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));

        _imu = new MPU6000(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);

        switch (_imu->begin()) {

            case MPUIMU::ERROR_IMU_ID:
                error("Bad device ID");
                break;
            case MPUIMU::ERROR_SELFTEST:
                error("Failed self-test");
                break;
            default:
                break;
        }
    }

    void Stm32FBoard::setLed(bool isOn)
    {
        ledSet(0, isOn);
    }

    bool Revo::imuRead(void)
    {
        if (_imu->checkNewData()) {  

            _imu->readAccelerometer(_ax, _ay, _az);
            _imu->readGyrometer(_gx, _gy, _gz);

            // Negate to support board orientation
            _ay = -_ay;
            _gx = -_gx;
            _gz = -_gz;

            return true;
        }  

        return false;
    }

} // extern "C"
