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
#include <math.h>

class Stats {

    private:

        float m_oldM;
        float  m_newM;
        float  m_oldS;
        float  m_newS;
        int32_t m_n;

        float variance(void)
        {
            return ((m_n > 1) ? m_newS / (m_n - 1) : 0.0f);
        }

    public:

        void clear(void)
        {
            m_n = 0;
        }

        void push(float x)
        {
            m_n++;

            if (m_n == 1) {
                m_oldM = m_newM = x;
                m_oldS = 0.0f;
            } else {
                m_newM = m_oldM + (x - m_oldM) / m_n;
                m_newS = m_oldS + (x - m_oldM) * (x - m_newM);
                m_oldM = m_newM;
                m_oldS = m_newS;
            }
        }

        float stdev(void)
        {
            return sqrtf(variance());
        }
}; 
