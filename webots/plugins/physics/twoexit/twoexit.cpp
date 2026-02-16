/* 
 * Custom physics plugin for two-exit autopilot simuation
 *
 * Copyright (C) 2026 Simon D. Levy
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

// Hackflight
#include <autopilots/twoexit.hpp>

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/world.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

static const char * LOG_FILE_NAME = "log.csv";

static const char * PATH_VARIABLE_NAME = "WEBOTS_PATH";
static const char * WORLD_VARIABLE_NAME = "WEBOTS_WORLD";

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

static simsens::World _world;

static simsens::Robot _robot;

static FILE * _logfile;

static PluginHelper * _helper;

static hf::TwoExitAutopilot _autopilot;

static char * worldname()
{
    return getenv(WORLD_VARIABLE_NAME);
}

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
            //const auto state = _helper->get_state_from_siminfo(siminfo);

            // Replace open-loop setpoint with setpoint from autopilot if
            // available
            if (siminfo.mode == hf::MODE_AUTONOMOUS) {
                static int _frame;
                _autopilot.getSetpoint(_frame++, siminfo.setpoint);
            }

            // Use setpoint to get new state
            const auto newstate = _helper->get_state_from_siminfo(siminfo);

            // Grab autopilot sensors for next iteration
            _autopilot.readSensors(_robot, _world,
                    {newstate.x, newstate.y, newstate.z,
                     newstate.phi, newstate.theta, newstate.psi},
                     _logfile);


            simsens::RangefinderVisualizer::show(
                    _autopilot.rangefinder_distances_mm,
                    _autopilot.get_rangefinder(_robot)->min_distance_m,
                    _autopilot.get_rangefinder(_robot)->max_distance_m,
                    8, 1, RANGEFINDER_DISPLAY_SCALEUP);

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

    sprintf(path, "%s/../../worlds/%s.wbt", pwd, worldname());
    simsens::WorldParser::parse(path, _world);

    sprintf(path, "%s/../../protos/DiyQuad.proto", pwd);
    simsens::RobotParser::parse(path, _robot);

    sprintf(path, "%s/%s", pwd, LOG_FILE_NAME);
    _logfile = fopen(path, "w");
}
