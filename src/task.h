/*
   Copyright (c) 2022 Simon D. Levy

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

class Task {

    public:

        Task(uint32_t rate)
        {
            m_desiredPeriodUs = 1000000 / rate;
        }

    private:

        int32_t  m_desiredPeriodUs;            
        uint32_t m_lastExecutedAtUs;          

        uint16_t m_dynamicPriority;          
        uint16_t m_taskAgeCycles;
        uint32_t m_lastSignaledAtUs;         
        uint32_t m_anticipatedExecutionTime;

        virtual void fun(hackflight_t * hf, uint32_t time) = 0;
};
