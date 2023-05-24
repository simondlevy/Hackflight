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

        static constexpr float MAX_DEMAND = 0.2;
        static constexpr float MAX_ANGLE = 45;
        static constexpr float MAX_SPEED = 10;
        static constexpr float WINDUP = 10;

        float m_k_p;
        float m_k_i;
        float m_k_d;

        float m_err_integral_y;
        float m_err_prev_y;

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

        void modifyDemand(
                const float speed,
                const uint32_t dusec,
                float & demand,
                float & err_prev,
                float & err_integral)
        {
            if (fabs(demand) > MAX_DEMAND) {
                return;
            }
            const auto du = dusec == 0 ? 1 : dusec;

            const auto err = -speed;

            const auto derr = m_k_p * (err - err_prev) / du;

            err_integral += m_k_i * (err * du);

            err_integral = err_integral < - WINDUP ? -WINDUP : 
                err_integral > WINDUP ? WINDUP :
                err_integral;

            demand = err + derr + err_integral;

            err_prev = err;
        }

    public:

        FlowHoldPidController(
                const float k_p = 0.0,
                const float k_i = 0.0000001,
                const float k_d = 0.01)
        {
            m_k_p = k_p;
            m_k_i = k_i;
            m_k_d = k_d;
        }

        virtual void modifyDemands(
                Demands & demands,
                const int32_t dusec,
                const VehicleState & vstate,
                const bool reset) override
        {
            (void)reset;

            /*
            printf("dt=%d | dx=%+3.3f  dy=%+3.3f | phi=%+3.3f  theta=%+3.3f\n", 
                    dusec, vstate.dx, vstate.dy, vstate.phi, vstate.theta);
                    */

            if (safeAngle(vstate.phi) && safeAngle(vstate.theta) &&
                    safeSpeed(vstate.dx) && safeSpeed(vstate.dy)) {
                modifyDemand(
                        vstate.dy,
                        dusec,
                        demands.roll,
                        m_err_prev_y,
                        m_err_integral_y);
                //modifyDemand(vstate.dx, dusec, demands.pitch, m_err_prev_x);
            }

        } // modifyDemands

}; // class FlowHoldPidController
