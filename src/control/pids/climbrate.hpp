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
#include <vehicles/diyquad.hpp>

class ClimbRateController {

    public:

        /**
         * Demand is input as climbrate target in meters per second and output
         * as arbitrary positive value to be scaled according to motor
         * characteristics.
         */
        static float run(
                const bool hovering,
                const float dt,
                const float z,
                const float dz,
                const float demand)
        {
            static float _integral;

            const auto airborne = hovering || (z > LANDING_ALTITUDE_METERS);

            const auto error = demand - dz;

            _integral = airborne ? 
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            const auto thrust = KP * error + KI * _integral;

            return airborne ?
                Num::fconstrain(thrust * THRUST_SCALE + THRUST_BASE,
                        THRUST_MIN, THRUST_MAX) : 0;
        }

    private:

        static constexpr float KP = 25;
        static constexpr float KI = 15;
        static constexpr float ILIMIT = 5000;

        static constexpr float LANDING_ALTITUDE_METERS = 0.03;
};
