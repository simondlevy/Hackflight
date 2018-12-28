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

    void Stm32FBoard::writeMotor(uint8_t index, float value)
    {
        motor_write(index, value);
    }

    /*
    void Stm32FBoard::delaySeconds(float sec)
    {
        delay((uint16_t)(sec*1000));
    }

    uint32_t Stm32FBoard::getMicroseconds(void)
    {
        return micros();
    }
    */

    void Stm32FBoard::reboot(void)
    {
        systemResetToBootloader();
    }

    bool Stm32FBoard::getQuaternion(float quat[4])
    {
        return SoftwareQuaternionBoard::getQuaternion(quat, getTime());
    }

    bool Stm32FBoard::getGyrometer(float gyroRates[3])
    {
        return SoftwareQuaternionBoard::getGyrometer(gyroRates);
    }

} // extern "C"
