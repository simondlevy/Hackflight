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

#include "esc.h"

class ArduinoBrushedEsc : public Esc {

    public:

        ArduinoBrushedEsc(vector<uint8_t> & pins)  
            : Esc(pins)
        {
        }

        virtual void begin(void) override
        {
            for (auto pin : *m_pins) {
                analogWriteFrequency(pin, 10000);
                analogWrite(pin, 0);
            }
        }

        virtual float  convertFromExternal(const uint16_t value) override 
        {
            return (value - PWM_MIN) / (float)(PWM_MAX - PWM_MIN);
        }

        virtual float getMotorValue(
                const float input, const bool failsafeIsActive) override
        {
            (void)failsafeIsActive;

            return constrain_f(input, 0, 1);
        }

        virtual void write(const float values[]) override 
        {
            uint8_t k=0;
            for (auto p: *m_pins) {
                analogWrite(p, (uint8_t)(values[k++] * 255));
            }
        }
};
