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

namespace hf {

    class PositionController {

        private:

            static constexpr float KP = 25; 
            static constexpr float KI = 1;
            static constexpr float ILIMIT = 5000;

            float _integral;

        public:

            static constexpr float MAX_DEMAND_DEG = 20;

            PositionController()
            {
                _integral = 0;
            }

            /**
             * Demands is input as normalized interval [-1,+1] and output as
             * angles in degrees.
             */
            float run(
                    const bool airborne,
                    const float dt,
                    const float target,
                    const float actual)
            {
                const auto error = target - actual;

                _integral = airborne ? 
                    Num::fconstrain(_integral + error * dt, ILIMIT) :
                    0;

                return airborne ?
                    Num::fconstrain(KP * error + KI * _integral, MAX_DEMAND_DEG) :
                    0;
            }

    };

}
