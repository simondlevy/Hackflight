/*
   fl3board.cpp : STM32F3 implementation of Hackflight Board routines

   Copyright (c) 2018 Simon D. Levy

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

#include "f3board.h"

namespace hf {

    void F3Board::delaySeconds(float sec)
    {
        (void)sec;
    }

    void F3Board::ledSet(bool is_on)
    { 
        (void)is_on;
    }

    uint8_t F3Board::serialAvailableBytes(void)
    {
        return 0;
    }

    uint8_t F3Board::serialReadByte(void)
    {
        return 0;
    }

    void F3Board::serialWriteByte(uint8_t c)
    {
        (void)c;
    }

    uint32_t F3Board::getMicroseconds(void)
    {
        return 0;
    }

    void F3Board::writeMotor(uint8_t index, float value)
    {
        (void)index;
        (void)value;
    }

    bool F3Board::getGyrometer(float gyroRates[3])
    {
        (void)gyroRates;
        return false;
    }

    bool F3Board::getQuaternion(float quat[4])
    {
        (void)quat;
        return false;
    }

    // Support prototype version where LED is on pin A1
    F3Board::F3Board(void)
    {
        // Do general real-board initialization
        RealBoard::init();
    }


} // namespace hf
