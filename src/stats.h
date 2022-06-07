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

#include <math.h>

#include "datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

    void devClear(stdev_t *dev)
    {
        dev->m_n = 0;
    }

    void devPush(stdev_t *dev, float x)
    {
        dev->m_n++;
        if (dev->m_n == 1) {
            dev->m_oldM = dev->m_newM = x;
            dev->m_oldS = 0.0f;
        } else {
            dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
            dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
            dev->m_oldM = dev->m_newM;
            dev->m_oldS = dev->m_newS;
        }
    }

    float devVariance(stdev_t *dev)
    {
        return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
    }

    float devStandardDeviation(stdev_t *dev)
    {
        return sqrtf(devVariance(dev));
    }

#if defined(__cplusplus)
}
#endif
