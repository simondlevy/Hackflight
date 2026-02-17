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

// Helpers
#include "../helper.hpp"
#include "../experimental.hpp"

// SimSensors
#include <simsensors/src/parsers/webots/world.hpp>
#include <simsensors/src/parsers/webots/robot.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

// Hackflight
#include <autopilots/twoexit.hpp>

static const char * PATH_VARIABLE_NAME = "WEBOTS_PATH";

static const char * LOG_FILE_NAME = "log.csv";

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

static simsens::World _world;
static simsens::Robot _robot;
static PluginHelper * _helper;
static ExperimentalHelper * _ehelper;

static hf::TwoExitAutopilot _autopilot;

static FILE * _logfile;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    static bool _collided;

    if (_collided) {
        dBodySetGravityMode(_helper->robotBody, 1);
    }

    else {

        PluginHelper::siminfo_t siminfo = {};

        if (_helper->get_siminfo(siminfo)) {

            // Replace open-loop setpoint with setpoint from autopilot if
            // available
            if (siminfo.mode == hf::MODE_AUTONOMOUS) {
                static int _frame;
                _autopilot.getSetpoint(_frame++, siminfo.setpoint);
            }

            // Use setpoint to get new state
            const auto state = _helper->get_state_from_siminfo(siminfo);

            // Extract pose from state
            const simsens::pose_t pose = {
                state.x, state.y, state.z, state.phi, state.theta, state.psi
            };

            // Read rangefinder distances for next iteration
            _autopilot.readSensor(_robot, _world, pose);

            // Log data to file
            _ehelper->write_to_log(
                    _logfile, pose, _autopilot.rangefinder_distances_mm, 8);

            // Display rangefinder distances
            simsens::RangefinderVisualizer::show(
                    _autopilot.rangefinder_distances_mm,
                    _autopilot.get_rangefinder(_robot)->min_distance_m,
                    _autopilot.get_rangefinder(_robot)->max_distance_m,
                    8, 1, RANGEFINDER_DISPLAY_SCALEUP);

            // Stop if we detected a collision
            if (_world.collided({state.x, state.y, state.z})) {
                _collided = true;
            }

            // Otherwise, set normally
            _helper->set_dbody_from_state(state);
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

    _ehelper = new ExperimentalHelper();

    const auto pwd = getenv(PATH_VARIABLE_NAME);

    char path[1000] = {};

    sprintf(path, "%s/../../worlds/twoexit.wbt", pwd);
    simsens::WorldParser::parse(path, _world);

    sprintf(path, "%s/../../protos/DiyQuad.proto", pwd);
    simsens::RobotParser::parse(path, _robot);

    sprintf(path, "%s/%s", pwd, LOG_FILE_NAME);
    _logfile = fopen(path, "w");
}
