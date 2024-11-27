/* 
 * Custom physics plugin for Hackflight simulator using Estimated Kalman Filter for
 * state and standard C++ PID controllers
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */


#include <sim/standard_controllers.hpp>

hf::state_t estimate_state(const hf::Dynamics & dynamics)
{
    return hf::state_t {
        dynamics._x1,
            dynamics._x2 * cos(dynamics._x11) -
                dynamics._x4 * sin(dynamics._x11),
        dynamics._x3,
        -(dynamics._x2 * sin(dynamics._x11) +
                    dynamics._x4 * cos(dynamics._x11)),
        dynamics._x5,
        dynamics._x6,
            hf::Utils::RAD2DEG* dynamics._x7,
            hf::Utils::RAD2DEG* dynamics._x8,
            hf::Utils::RAD2DEG* dynamics._x9,
            hf::Utils::RAD2DEG* dynamics._x10,
            hf::Utils::RAD2DEG* dynamics._x11,
            hf::Utils::RAD2DEG* dynamics._x12,
    };
}
