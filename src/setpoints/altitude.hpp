/**
 *
 * Copyright (C) 2026 Simon D. Levy
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

class AltitudeSetpoint {

    public:

        /**
          *  @param dt time constant
          *  @param demand in [-1,+1]
          *  @return altitude setpoint in meters
          */

        static float run(const float dt, const float demand)
        {
            (void)dt;
            (void)demand; 

            static float _target;

            return _target;
        }

    private:

        static constexpr float HOVER_INIT_M = 0.4;
        static constexpr float HOVER_MAX_M = 1.0;
        static constexpr float HOVER_MIN_M = 0.2;
};
