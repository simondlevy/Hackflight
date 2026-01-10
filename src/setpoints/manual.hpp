/**
 * Setpoint from manual controller (gamepad / keyboard)
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

class ManualSetpoint {

    public:

        /**
          *  @param dt time constant
          *  @setpoint input/output setpoint
          *
          *  Setpoint is modified by accmulating altitude target and yaw angle
          */

        static void run(const float dt, demands_t & setpoint)
        {
            static float _altitude_target;

            if (_altitude_target == 0) {
                _altitude_target = ALTITUDE_INIT_M;
            }

            _altitude_target = Num::fconstrain(
                        _altitude_target + setpoint.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

            setpoint.thrust = _altitude_target;

            static float _yaw_angle_target;

            _yaw_angle_target = Num::cap_angle(
                    _yaw_angle_target + YAW_DEMAND_MAX * setpoint.yaw * dt);

            setpoint.yaw = _yaw_angle_target;
        }

    private:

        static constexpr float ALTITUDE_INIT_M = 0.4;
        static constexpr float ALTITUDE_MAX_M = 1.0;
        static constexpr float ALTITUDE_MIN_M = 0.2;
        static constexpr float ALTITUDE_INC_MPS = 0.2;

        static constexpr float YAW_DEMAND_MAX = 200;
};
