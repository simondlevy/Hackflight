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

    class YawAngleController {

        public:

            /**
             *  @param dt time constant
             *  @param psi_actual current heading in degrees
             *  @param psi_target target heading in degrees
             *  @return yaw demand in deg/sec
             */

            static float run(
                    const bool airborne, 
                    const float dt,       
                    const float psi_actual,
                    const float psi_target)
            {
                // Grab initial psi first time around
                static float _psi_initial;
                if (_psi_initial == 0) {
                    _psi_initial = psi_actual;
                }

                static float _integral;
                static float _previous;

                const auto error =
                    Num::cap_angle(psi_target - (psi_actual - _psi_initial));

                _integral = airborne ?
                    Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

                auto deriv = dt > 0 ? (error - _previous) / dt : 0;

                _previous = error;

                return KP * error + KI * _integral + KD * deriv;
            }

        private:

            static constexpr float KP = 6;
            static constexpr float KI = 1;
            static constexpr float KD = 0.35;
            static constexpr float ILIMIT = 360;

            float _integral;
            float _previous;

    };
}
