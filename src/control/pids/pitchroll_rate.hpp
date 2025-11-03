/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <num.hpp>

class PitchRollRateController {

    public:

        /**
          * Demands are input as angular velocities in degrees per second and
          * output as as arbitrary values to be scaled according to motor
          * characteristics:
          *
          * roll:  input roll-right positive => output positive
          *
          * pitch: input nose-down positive => output positive
          */
         static void run(
                 const bool airborne,
                 const float dt,
                 const float state_dphi, const float state_dtheta,
                 const float demand_roll,const float demand_pitch, 
                 float & new_demand_roll, float & new_demand_pitch) 
         {
             static axis_t _roll;

             static axis_t _pitch;

             new_demand_roll =
                 runAxis(airborne, dt, demand_roll, state_dphi, _roll);

             new_demand_pitch =
                 runAxis(airborne, dt, demand_pitch, state_dtheta, _pitch);
         }

    private:

        static constexpr float KP = 125 / 2;
        static constexpr float KI = 250 / 2;
        static constexpr float KD = 1.25 / 2;
        static constexpr float ILIMIT = 33;

        typedef struct {
            float integral;
            float previous;
        } axis_t;

        static float runAxis(
                const bool airborne,
                const float dt,
                const float demand,
                const float measured,
                axis_t & axis)
        {
            const auto error = demand - measured;

            axis.integral = airborne ?
                Num::fconstrain( axis.integral + error * dt, ILIMIT) : 0;

            auto deriv = dt > 0 ? (error - axis.previous) / dt : 0;

            axis.previous = airborne ? error : 0;

            return airborne ? KP * error + KI * axis.integral + deriv : 0;
        }
};
