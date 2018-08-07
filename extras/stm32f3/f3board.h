/*
   fl3board.h : STM32F3 implementation of Hackflight Board routines

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

#pragma once

#include <boards/realboard.hpp>

namespace hf {

    class F3Board : public RealBoard {

        //protected:
        public:

            void delaySeconds(float sec);

            void ledSet(bool is_on);

            uint8_t serialAvailableBytes(void);

            uint8_t serialReadByte(void);

            void serialWriteByte(uint8_t c);

            virtual uint32_t getMicroseconds(void) override;

            void writeMotor(uint8_t index, float value);

            bool getGyrometer(float gyroRates[3]);

            bool getQuaternion(float quat[4]);

        public:

            F3Board(void);

    }; // class F3Board

} // namespace hf
