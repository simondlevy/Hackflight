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

#include <math.h>

#include "pid.h"

// PT2 Low Pass filter
class Pt2Filter {

    private:

        float m_state;
        float m_state1;
        float m_dt;
        float m_k;

    public:

        Pt2Filter(const float f_cut, const float dt=PidController::DT)
        {
            m_state = 0.0;
            m_state1 = 0.0;
            m_dt = dt;

            computeGain(f_cut);
        }

        float apply(const float input)
        {
            m_state1 = m_state1 + m_k * (input - m_state1);
            m_state = m_state + m_k * (m_state1 - m_state);
            return m_state;
         }

        void computeGain(const float f_cut)
        {
            const float order = 2.0f;
            const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
            float rc = 1 / (2 * orderCutoffCorrection * M_PI * f_cut);
            m_k = m_dt / (rc + m_dt);
         }

}; // class Pt2Filter
