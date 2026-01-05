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

class YawAngleSetpoint {

    public:

        /**
          *  @param dt time constant
          *  @param yaw_demand yaw demand, as fraction of a max turn rate
          *  @return yaw demand in degrees
          */

        static float run(const float dt, const float yaw_demand)
        {
            static float _target;

            _target = Num::cap_angle(_target + DEMAND_MAX * yaw_demand * dt);

            printf("setpoint: %+3.3f\n", _target);

            return _target;
        }

    private:

        static constexpr float DEMAND_MAX = 200;
};
