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

// C
#include <stdlib.h>
#include <unistd.h>

// Webots
#include "../helper.hpp"

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/world.hpp>

// Hackflight
#include <simulator/dynamics.hpp>
#include <autopilots/pingpong.hpp>

static const char * PATH_VARIABLE_NAME = "WEBOTS_PATH";

static simsens::World _world;

static simsens::Robot _robot;

static PluginHelper * _helper;

static hf::PingPongAutopilot _autopilot;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    static bool _collided;

    if (_collided) {
        dBodySetGravityMode(_robotBody, 1);
    }

    else {

        PluginHelper::siminfo_t siminfo = {};

        if (_helper->get_siminfo(siminfo)) {

            // Get current vehicle state
            const auto state = _helper->get_state_from_siminfo(siminfo);

            // Replace open-loop setpoint with setpoint from autopilot if
            // available
            if (siminfo.mode == hf::MODE_AUTONOMOUS) {
                _autopilot.getSetpoint(state, siminfo.setpoint);
            }

            // Use setpoint to get new state
            const auto newstate = _helper->get_state_from_siminfo(siminfo);

            // Grab rangefinder readings for next iteration
            _autopilot.readSensors(_robot, _world, newstate);

            // Stop if we detected a collision
            if (_world.collided({newstate.x, newstate.y, newstate.z})) {
                _collided = true;
            }

            // Otherwise, set normally
            _helper->set_dbody_from_state(newstate);
        }
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

DLLEXPORT void webots_physics_init() 
{
    _helper = new PluginHelper();

    const auto pwd = getenv(PATH_VARIABLE_NAME);

    char path[1000] = {};

    sprintf(path, "%s/../../worlds/pingpong.wbt", pwd);
    simsens::WorldParser::parse(path, _world);

    sprintf(path, "%s/../../protos/DiyQuad.proto", pwd);
    simsens::RobotParser::parse(path, _robot);
}
