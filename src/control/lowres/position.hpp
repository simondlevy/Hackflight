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

class PositionController {

    public:

        /**
          * Demands are input as normalized interval [-1,+1] and output as
          * angles in degrees.
          *
          * roll:  input left positive => output negative
          *
          * pitch: input forward positive => output negative
          */
         static void run(
                 const bool airborne,
                 const float dt,
                 const float state_dx,
                 const float state_dy,
                 const float state_psi,
                 const float demand_x,
                 const float demand_y,
                 float & roll,
                 float & pitch)
        {
            static float _integralX;
            static float _integralY;

            // Rotate world-coordinate velocities into body coordinates
            const auto dxw = state_dx;
            const auto dyw = state_dy;
            const auto psi = Num::DEG2RAD * state_psi;
            const auto cospsi = cos(psi);
            const auto sinpsi = sin(psi);
            const auto dxb =  dxw * cospsi + dyw * sinpsi;
            const auto dyb = -dxw * sinpsi + dyw * cospsi;       

            // Run PIDs on body-coordinate velocities
            roll = runAxis(airborne, dt, demand_y, dyb, _integralY);
            pitch =runAxis(airborne, dt, demand_x, dxb, _integralX);
        }

    private:

        static constexpr float KP = 25; 
        static constexpr float KI = 1;
        static constexpr float ILIMIT = 5000;
        static constexpr float LIMIT = 20;
        static constexpr float LIMIT_OVERHEAD = 1.10;

        static float runAxis(
                const bool airborne,
                const float dt,
                float demand,
                const float measured, 
                float & integral)
        {
            const auto error = demand - measured;

            integral = airborne ? 
                Num::fconstrain(integral + error * dt, ILIMIT) : 0;

            return airborne ?
                Num::fconstrain(KP * error + KI * integral, LIMIT) : 0;
        }
};
