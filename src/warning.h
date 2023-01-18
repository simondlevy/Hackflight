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

#include <stdbool.h>

class Warning {

    public:

        typedef enum {

            OFF,
            ON,
            BLINK

        } state_e;

        state_e state;

        uint32_t timer;

        bool ledOn;

        void setTimer(const uint32_t usec)
        {
            timer = usec + 500000;
        }

        void toggleLed(void)
        {
            ledOn = !ledOn;
        }

        void disable(void)
        {
            state = OFF;
        }

        void blink(void)
        {
            state = Warning::BLINK;
        }

}; // class Warning
