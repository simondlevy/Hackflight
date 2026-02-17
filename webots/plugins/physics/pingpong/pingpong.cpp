/* 
 * Custom physics plugin for ping-pong autopilot simuation
 *
 *  Copyright (C) 2025 Simon D. Levy
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

#include "../helper.hpp"
#include "../experimental.hpp"

#include <autopilots/pingpong.hpp>

static ExperimentalHelper * _ehelper;

static hf::PingPongAutopilot _autopilot;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    PluginHelper::siminfo_t siminfo = {};

    if (_ehelper->get_siminfo(siminfo)) {

        // Start in a random direction (forward or backward)
        static bool _started;
        if (!_started) {
            siminfo.setpoint.pitch = 2 * (rand() % 2) - 1;
        }
        _started = true;

        // Get current vehicle state
        const auto state = _ehelper->get_state_from_siminfo(siminfo);

        // Replace open-loop setpoint with setpoint from autopilot if
        // available
        if (siminfo.mode == hf::MODE_AUTONOMOUS) {
            _autopilot.getSetpoint(state.dy, siminfo.setpoint);
        }

        // Get vehicle pose based on setpoint
        const auto pose = _ehelper->get_pose(siminfo);

        // Grab rangefinder readings for next iteration
        _autopilot.readSensors(_ehelper->robot, _ehelper->world, pose);

        // Log data to file
        const int distances[] = {
            _autopilot.distance_forward_mm, 
            _autopilot.distance_backward_mm};
        _ehelper->write_to_log(pose, distances, 2);
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _ehelper;
}

DLLEXPORT void webots_physics_init() 
{
    _ehelper = new ExperimentalHelper("pingpong");
}
