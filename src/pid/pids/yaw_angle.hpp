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

class YawAngleController {

    public:

        /**
          *  @param dt time constant
          *  @param psi current heading in degrees
          *  @param yaw yaw demand, expressed as fraction of a max turn rate
          *  @return yaw demand in deg/sec
          */

        static float run(
                const float dt,       
                const float psi,
                const float yaw)
        {
            // Grab initial psi first time around
            static float _psi_initial;
            if (_psi_initial == 0) {
                _psi_initial = psi;
            }

            static float _target;
            static float _integral;
            static float _previous;

            _target = Num::cap_angle(_target + DEMAND_MAX * yaw * dt);

            printf("pid: %+3.3f\n", _target);

            const auto error = Num::cap_angle(_target - (psi - _psi_initial));

            _integral = Num::fconstrain(_integral + error * dt, ILIMIT);

            auto deriv = dt > 0 ? (error - _previous) / dt : 0;

            _previous = error;

            return KP * error + KI * _integral + KD * deriv;
        }

    private:

        static constexpr float KP = 6;
        static constexpr float KI = 1;
        static constexpr float KD = 0.35;
        static constexpr float ILIMIT = 360;
        static constexpr float DEMAND_MAX = 200;

        float _integral;
        float _previous;

};
