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

#include <vector>

class Esc {

    friend class Stm32Board;

    protected:

        typedef enum {

            MOCK,
            BRUSHED,
            DSHOT

        } type_e;

        type_e type;

        std::vector<uint8_t> * motorPins;

        uint32_t dshotOutputFreq;

        Esc(type_e type, std::vector<uint8_t> &motorPins)
        {
            this->type = type;
            this->motorPins = &motorPins;
        }

        Esc(void)
        {
            this->type = MOCK;
        }

        virtual uint16_t prepareDshotPacket(const uint8_t i, const float motorValue)
        {
            (void)i;
            (void)motorValue;
            return 0;
        }

        virtual void dshotComplete(void) 
        {
        }
 
    public:

        virtual float convertFromExternal(const uint16_t value)
        {
            (void)value;
            return 0;
        }

        virtual float getMotorValue(const float input)
        {
            (void)input;
            return 0;
        }

        virtual bool  isReady(const uint32_t currentTime) 
        {
            (void)currentTime;
            return true;
        }

        virtual void  stop(void) 
        {
        }
};
