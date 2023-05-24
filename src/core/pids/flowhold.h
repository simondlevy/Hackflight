/*
   Copyright (c) 2023 Simon D. Levy

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

#include <string.h>
#include <math.h>

#include "core/constrain.h"
#include "core/filters/pt1.h"
#include "core/filters/pt2.h"
#include "core/pid.h"
#include "core/utils.h"

class FlowHoldPidController : public PidController {

    private:

        static constexpr float MAX_ANGLE = 5;
        static constexpr float MAX_SPEED = 1;

        float m_k_rate_p;
        float m_k_rate_i;
        float m_k_rate_d;

        static bool safe(const float val, const float max)
        {
            return fabs(val) < max;
        }

        static bool safeAngle(const float angle)
        {
            return safe(angle, MAX_ANGLE);
        }

        static bool safeSpeed(const float speed)
        {
            return safe(speed, MAX_SPEED);
        }

    public:

        FlowHoldPidController(
                const float k_rate_p = 0,
                const float k_rate_i = 0,
                const float k_rate_d = 0)
        {
            m_k_rate_p = k_rate_p;
            m_k_rate_i = k_rate_i;
            m_k_rate_d = k_rate_d;
        }

        virtual void modifyDemands(
                Demands & demands,
                const int32_t dusec,
                const VehicleState & vstate,
                const bool reset) override
        {

            (void)demands;
            (void)dusec;
            (void)reset;

            if (safeAngle(vstate.phi) && safeAngle(vstate.theta) &&
                    safeSpeed(vstate.dx) && safeSpeed(vstate.dy)) {
                printf("dx=%+3.3f  dy=%+3.3f | phi=%+3.3f  theta=%+3.3f\n", 
                        vstate.dx, vstate.dy, vstate.phi, vstate.theta);
            }

        } // modifyDemands

}; // class FlowHoldPidController
