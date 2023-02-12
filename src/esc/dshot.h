/*
   Copyright (c) 2023 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <vector>

#include <stm32dshot.h>

#include "esc.h"

class DshotEsc : public Esc {

    private:

        Stm32Dshot * m_dshot;

    public:

        DshotEsc(Stm32Dshot * dshot, std::vector<uint8_t> * motorPins)
            : Esc(motorPins)
        {
            m_dshot = dshot;
        }

        virtual void begin(void) override
        {
            m_dshot->begin(*m_motorPins);
        }

        virtual void write(float motorValues[]) override
        {
            m_dshot->write(motorValues);
        }
}; 
