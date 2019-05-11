/*
   furyf4.cpp : Board implmentation for FuryF4

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

#include "furyf4.h"
#include "mpu6000spi.h"

// Here we put code that interacts with Cleanflight
extern "C" {

// Cleanflight includes
#include "drivers/light_led.h"

// Hackflight includes
#include "../../common/spi.h"
#include "../../common/motors.h"

// Hackflight includes
#include "../../common/spi.h"
#include "../../common/beeperled.h"
#include "../../common/motors.h"
#include <debugger.hpp>

    // Required by system_stm32f4xx.c
    void * mpuResetFn = NULL;

    // We put this outside the class to make it available to static Board::outbuf() below
    static serialPort_t * _serial0;

    FuryF4::FuryF4(void)
    {
        _serial0 = usbVcpOpen();

        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));

        RealBoard::init();
    }

    void FuryF4::setLed(bool isOn)
    {
        ledSet(1, isOn);
    }

    int16_t accx, accy, accz;

    bool FuryF4::imuReady(void)
    {
        accx = 7;
        accy = 8;
        accz = 9;

        return false;
    }


    void FuryF4::imuReadAccelGyro(void)
    {
    }
    
    void FuryF4::writeMotor(uint8_t index, float value)
    {
        (void)index;
        (void)value;
    }

    void FuryF4::reboot(void)
    {
        systemResetToBootloader();
    }

    bool FuryF4::getQuaternion(float & qw, float & qx, float & qy, float & qz)
    {
        return SoftwareQuaternionBoard2::getQuaternion(qw, qx, qy, qz, getTime());
    }

    bool FuryF4::getGyrometer(float & gx, float & gy, float & gz)
    {
        return SoftwareQuaternionBoard2::getGyrometer(gx, gy, gz);
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

    uint8_t FuryF4::serialNormalAvailable(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t FuryF4::serialNormalRead(void)
    {
        return serialRead(_serial0);
    }

    void FuryF4::serialNormalWrite(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    void FuryF4::adHocDebug(void)
    {
        hf::Debugger::printf("%d %d %d\n", accx, accy, accz);
    }

} // extern "C"
