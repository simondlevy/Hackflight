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
#include "../autopilot.hpp"

#include <sim/autopilots/pingpong.hpp>

static AutopilotHelper * _helper;

static hf::PingPongAutopilot _autopilot;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    PluginHelper::siminfo_t siminfo = {};

    if (_helper->get_siminfo(siminfo)) {

        // Get current vehicle state
        const auto state = _helper->get_state_from_siminfo(siminfo);

        // Replace open-loop setpoint with setpoint from autopilot if
        // available
        if (siminfo.mode == hf::MODE_AUTONOMOUS) {
            _autopilot.getSetpoint(state.dy, siminfo.setpoint);
        }

        // Get vehicle pose based on setpoint
        const auto pose = _helper->get_pose(siminfo);

        // Grab rangefinder readings for next iteration
        _autopilot.readSensors(_helper->robot, _helper->world, pose);

        // Log data to file
        const int distances[] = {
            _autopilot.distance_forward_mm, 
            _autopilot.distance_backward_mm};
        _helper->write_to_log(pose, distances, 2);
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

DLLEXPORT void webots_physics_init() 
{
    hf::PingPongAutopilot::init();

    _helper = new AutopilotHelper("pingpong");
}
