/*
   stm32fboard.cpp : Board implementation for STM32F boards

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

#include "stm32fboard.h"

// Here we put code that interacts with Cleanflight
extern "C" {

    // Hackflight includes
#include "../../common/spi.h"
#include "../../common/beeperled.h"
#include "../../common/motors.h"

    // We put this outside the class to make it available to static Board::outbuf() below
    static serialPort_t * _serial0;

    Stm32FBoard::Stm32FBoard(serialPort_t * serial0)
    {
        _serial0 = serial0;
    }

    void Stm32FBoard::checkImuError(MPUIMU::Error_t errid)
    {
        switch (errid) {

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

    void Stm32FBoard::writeMotor(uint8_t index, float value)
    {
        motor_write(index, value);
    }

    void Stm32FBoard::reboot(void)
    {
        systemResetToBootloader();
    }

    bool Stm32FBoard::getQuaternion(float & qw, float & qx, float & qy, float & qz)
    {
        return SoftwareQuaternionBoard::getQuaternion(qw, qx, qy, qz, getTime());
    }

    bool Stm32FBoard::getGyrometer(float gyroRates[3])
    {
        return SoftwareQuaternionBoard::getGyrometer(gyroRates);
    }

    void hf::Board::outbuf(char * buf)
    {
        for (char *p=buf; *p; p++)
            serialWrite(_serial0, *p);
    }

    uint8_t Stm32FBoard::serialNormalAvailable(void)
    {
        return serialRxBytesWaiting(_serial0);
    }

    uint8_t Stm32FBoard::serialNormalRead(void)
    {
        return serialRead(_serial0);
    }

    void Stm32FBoard::serialNormalWrite(uint8_t c)
    {
        serialWrite(_serial0, c);
    }

    void Stm32FBoard::getRawImu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz)
    {
        hf::SoftwareQuaternionBoard::getRawImu(ax, ay, az, gx, gy, gz);
    } 

} // extern "C"
