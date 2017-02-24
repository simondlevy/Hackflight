/*
   extras.hpp : Class declaration for extra functionality (altitude hold, hover, etc.)

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

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include <stdint.h>

class Extras {

    public:

        void    init(void);

        void    checkSwitch(void);

        uint8_t getTaskCount(void);

        bool    handleMSP(uint8_t command);

        void    performTask(uint8_t taskIndex);
};

#ifdef __arm__
} // extern "C"
#endif
