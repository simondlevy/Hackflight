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

// Hackflight
#include <datatypes.h>
#include "../helper.hpp"
#include "../autopilot.hpp"

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>
#include <simsensors/src/visualizers/rangefinder.hpp>

static const uint8_t RANGEFINDER_DISPLAY_SCALEUP = 64;
static constexpr float FRAME_RATE_HZ = 32;

static auto getSetpoint(const int * rangefinder_distances_mm) -> hf::Setpoint
{
    const int * d = rangefinder_distances_mm;

    // Look for clear (infinity reading) in center of 1x8 readings
    const bool center_is_clear = d[3] == -1 && d[4] == -1;

    // If clear, pitch forward
    const auto pitch = center_is_clear ? 0.4 : 0;

    // Otherwise, yaw rightward
    const auto yaw = center_is_clear ? 0 : 0.2;

    return hf::Setpoint(0, 0, pitch, yaw);
}        

static AutopilotHelper * _helper;

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    PluginHelper::siminfo_t siminfo = {};

    if (_helper->get_siminfo(siminfo)) {

        static int _rangefinder_distances_mm[8];

        // Replace open-loop setpoint with setpoint from autopilot if
        // available
        siminfo.setpoint = siminfo.mode == hf::MODE_AUTONOMOUS ?
            getSetpoint(_rangefinder_distances_mm) : siminfo.setpoint;

        // Get vehicle pose based on setpoint
        const auto pose = _helper->get_pose(siminfo);

        auto rangefinder = _helper->robot.rangefinders["VL53L5-forward"];

        // Grab rangefinder distances for next iteration
        rangefinder.read(pose, _helper->world, _rangefinder_distances_mm);

        // Log data to file
        _helper->write_to_log(pose, _rangefinder_distances_mm, 8);

        // Display rangefinder distances
        simsens::RangefinderVisualizer::show(
                _rangefinder_distances_mm,
                rangefinder.min_distance_m,
                rangefinder.max_distance_m,
                8, 1, RANGEFINDER_DISPLAY_SCALEUP);
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

DLLEXPORT void webots_physics_init() 
{
    _helper = new AutopilotHelper("twoexit");
}
