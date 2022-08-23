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

#include "filters.h"

static float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

static void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

static float pt1FilterGain(float f_cut, float dT)
{
    float RC = 1 / (2 * M_PI * f_cut);
    return dT / (RC + dT);
}

// PT1 Low Pass filter
class Pt1Filter {

    private:

        float m_state;
        float m_k;

    public:

        Pt1Filter(float k)
        {
            m_state = 0.0;
            m_k = k;
        }

        float apply(float input)
        {
            m_state = m_state + m_k * (input - m_state);
            return m_state;
        }

        static float gain(float f_cut, float dT)
        {
            float RC = 1 / (2 * M_PI * f_cut);
            return dT / (RC + dT);
        }

}; // class Pt1Filter
