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

#include <bytescaling.hpp>
#include <num.hpp>

class AltitudeController {

    public:

        static constexpr float ZMIN = 0;
        static constexpr float ZMAX = 3;

        static constexpr float THRUSTMIN = 0;
        static constexpr float THRUSTMAX = 1;

        /**
         * Demand is input as altitude target in meters and output as 
         * climb rate in meters per second.
         */
        static float run(
                const bool hovering,
                const float dt,
                const uint8_t z,
                const uint8_t thrust)
        {
            static float _integral;

            const float thrustf = ByteScaling::byte2float(thrust, THRUSTMIN, THRUSTMAX );

            const float zf = ByteScaling::byte2float(z , ZMIN, ZMAX);

            const auto error = thrustf - zf;
                
            _integral = hovering ?
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            const float output = 
                hovering ? 
                Num::fconstrain(KP * error + KI * _integral,
                        fmaxf(VEL_MAX, 0.5f)  * VEL_MAX_OVERHEAD) :
                -LANDING_SPEED_MPS;

            return output;
        }

    private:

        static constexpr float KP = 2;
        static constexpr float KI = 0.5;
        static constexpr float ILIMIT = 5000;
        static constexpr float VEL_MAX = 1;
        static constexpr float VEL_MAX_OVERHEAD = 1.10;
        static constexpr float LANDING_SPEED_MPS = 0.15;
};
