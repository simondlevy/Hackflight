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

#include "../helper.hpp"
#include "../experimental.hpp"

#include <autopilots/twoexit.hpp>

#include <simsensors/src/visualizers/rangefinder.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;

static hf::TwoExitAutopilot _autopilot;

static ExperimentalHelper * _ehelper;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    if (!_ehelper->collided()) {

        PluginHelper::siminfo_t siminfo = {};

        if (_ehelper->get_siminfo(siminfo)) {

            // Replace open-loop setpoint with setpoint from autopilot if
            // available
            if (siminfo.mode == hf::MODE_AUTONOMOUS) {
                static int _frame;
                _autopilot.getSetpoint(_frame++, siminfo.setpoint);
            }

            const auto pose = _ehelper->get_pose(siminfo);

            // Grab rangefinder distances for next iteration
            _autopilot.readSensor(_ehelper->robot, _ehelper->world, pose);

            // Log data to file
            _ehelper->write_to_log(
                    pose, _autopilot.rangefinder_distances_mm, 8);

            // Display rangefinder distances
            simsens::RangefinderVisualizer::show(
                    _autopilot.rangefinder_distances_mm,
                    _autopilot.get_rangefinder(_ehelper->robot)->min_distance_m,
                    _autopilot.get_rangefinder(_ehelper->robot)->max_distance_m,
                    8, 1, RANGEFINDER_DISPLAY_SCALEUP);
        }
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _ehelper;
}

DLLEXPORT void webots_physics_init() 
{
    _ehelper = new ExperimentalHelper("twoexit");
}
