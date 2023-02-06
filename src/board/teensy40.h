/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <Arduino.h>

#include "board.h"

class Teensy40 : public Board {

    protected:

        virtual uint32_t getCycleCounter(void) override
        {
            return ARM_DWT_CYCCNT;
        }

        virtual uint32_t getClockSpeed(void) override
        {
            return F_CPU;
        }

        virtual void startCycleCounter(void) override
        {
            ARM_DEMCR |= ARM_DEMCR_TRCENA;
            ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
        }

    public:

        Teensy40(
                Imu & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const int8_t ledPin)
            : Board(&imu, pids, mixer, esc, ledPin)
        {
        }


};  //class Teensy40
