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

    private:

        uint8_t m_pins[MAX_SUPPORTED_MOTORS];
        uint8_t m_count;

    public:

        BrushedEsc(uint8_t * pins, uint8_t count) override 
        {
            for (auto k=0; k<count; ++k) {
                m_pins[k] = pins[k];
            }

            m_count = count;
        }

        virtual void begin(void) override
        {
            for (auto k=0; k<m_count; ++k) {
                analogWriteFrequency(m_pins[k], 10000);
                analogWrite(m_pins[k], 0);
            }
        }

        virtual float  convertFromExternal(void * device, uint16_t value) override 
        {
            (void)device;
            (void)value;

            return (value - PWM_MIN) / (float)(PWM_MAX - PWM_MIN);
        }

        virtual bool isProtocolDshot(void) override 
        {
            return false;
        }

        virtual bool isReady(uint32_t currentTime) override 
        {
            (void)currentTime;

            return true;
        }

        virtual float valueDisarmed(void) override 
        {
            return 0;
        }

        virtual float valueHigh(void) override 
        {
            return 1;
        }

        virtual float valueLow(void) override 
        {
            return 0;
        }

        virtual void stop(void) override 
        {
        }

        virtual void write(float *values) override 
        {
            for (auto k=0; k<m_count; ++k) {
                analogWrite(m_pins[k], (uint8_t)(m_values[k] * 255));
            }
        }
};
