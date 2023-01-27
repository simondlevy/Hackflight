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

#include "board.h"

class Stm32Board : public Board {

    public:

        virtual uint32_t getCycleCounter(void) override
        {
            return DWT->CYCCNT;
        }

    protected:

        Stm32Board(
                Imu * imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const int8_t ledPin)
            : Board(imu, pids, mixer, esc, ledPin)
        {
        }

        virtual uint32_t getClockSpeed(void) override
        {
            return SystemCoreClock;
        }

     private:


        void (*uartFun)(void);

        virtual void startCycleCounter(void) override
        {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

            __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
            *(DWTLAR) = 0xC5ACCE55;

            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }
};
