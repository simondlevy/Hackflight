/* 
 * Custom physics plugin for Hackflight simulator using ground-truth 
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
#include <sim/sensors/groundtruth.hpp>

// Called by webots_physics_init(); unneeded here
void setup_controllers()
{
}

// Called by webots_physics_step()
hf::state_t estimate_state(
        const hf::Dynamics & dynamics, const float pid_rate)
{
    (void)pid_rate;

    return hf::GroundTruth::read(dynamics);
}
