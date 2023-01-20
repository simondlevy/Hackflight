/*
   Copyright (c) 2022 Simon D. Levy

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

#include <stdint.h>
#include <stdbool.h>

class Esc {

    protected:

        typedef enum {

            MOCK,
            BRUSHED,
            DSHOT

        } type_e;

        type_e type;

        Esc(type_e type)
        {
            this->type = type;
        }

    public:

        class Board * board;

        virtual void  begin(void) = 0;

        virtual void  write(const float values[]) = 0;

        virtual float getMotorValue(const float input) = 0;

        virtual float convertFromExternal(const uint16_t value) = 0;

        virtual bool  isReady(const uint32_t currentTime) 
        {
            (void)currentTime;
            return true;
        }

        virtual void  stop(void) 
        {
        }
};
