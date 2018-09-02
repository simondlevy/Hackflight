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

    static serialPort_t * _serial0;

    Revo::Revo(void)
    {
        initMotors();
        initUsb();
        initImu();

        RealBoard::init();
    }

    void Revo::initImu(void)
    {
        spi_init(MPU6000_SPI_INSTANCE, IOGetByTag(IO_TAG(MPU6000_CS_PIN)));
        _imu = new MPU6000(AFS_2G, GFS_250DPS);

        switch (_imu->begin()) {

            case MPU_ERROR_IMU_ID:
                error("Bad device ID");
                break;
            case MPU_ERROR_SELFTEST:
                error("Failed self-test");
                break;
            default:
                break;
        }
    }

    void Revo::initUsb(void)
    {
        _serial0 = usbVcpOpen();
    }

    void Revo::initMotors(void)
    {

    }

    void Revo::writeMotor(uint8_t index, float value)
    {
        (void)index;
        (void)value;
    }

    void Revo::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    void Revo::setLed(bool isOn)
    {
        ledSet(0, isOn);
    }

    uint32_t Revo::getMicroseconds(void)
    {
        return micros();
    }

    void Revo::reboot(void)
    {
        systemResetToBootloader();
    }

    uint8_t Revo::serialAvailableBytes(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t Revo::serialReadByte(void)
    {
        return serialRead(_serial0);
    }

    void Revo::serialWriteByte(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    bool Revo::imuRead(void)
    {
        return false;
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

} // extern "C"
