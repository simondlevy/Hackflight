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

// Hackflight
#include <datatypes.h>
#include <sim/dynamics.hpp>
#include "../helper.hpp"
#include "../autopilot.hpp"

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static constexpr int DISTANCE_DIFFERENCE_THRESHOLD = 1800;
static constexpr float SPEED = 0.5;

static AutopilotHelper * _helper;

static int readRangefinder(
        const string name,
        simsens::Robot & robot,
        simsens::World & world,
        const simsens::Pose & pose)
{
    auto rangefinder = robot.rangefinders[name];

    int distance_mm = 0;

    rangefinder.read(pose, world, &distance_mm);

    return distance_mm;
}

static auto getSetpoint(
        const int distance_forward_mm,
        const int distance_backward_mm,
        const float dydt) -> hf::Setpoint
{
    const auto diff = distance_forward_mm - distance_backward_mm;

    const int8_t direction = 

        // (Nearly) motionless on startup; move in a random
        // direction (forward or backward)
        fabs(dydt) < 1e-6 ? 2 * (rand() % 2) - 1 :

        // Too close to either wall; switch direction
        diff > DISTANCE_DIFFERENCE_THRESHOLD ? +1 :
        diff < -DISTANCE_DIFFERENCE_THRESHOLD ? - 1 :

        // Otherwise, continue in same direction
        dydt > 0 ? -1 : +1;

    return hf::Setpoint(0, 0, direction * SPEED, 0);
}

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    PluginHelper::siminfo_t siminfo = {};

    if (_helper->get_siminfo(siminfo)) {

        // Get current vehicle state
        const auto state = _helper->get_state_from_siminfo(siminfo);

        static int _distance_forward_mm;
        static int _distance_backward_mm;

        // Replace open-loop setpoint with setpoint from autopilot if
        // available
        siminfo.setpoint = siminfo.mode == hf::MODE_AUTONOMOUS ?
            getSetpoint(_distance_forward_mm, 
                    _distance_backward_mm, state.dy) :
            siminfo.setpoint;

        // Get vehicle pose based on setpoint
        const auto pose = _helper->get_pose(siminfo);

        // Grab rangefinder readings for next iteration
        _distance_forward_mm = readRangefinder("VL53L1-forward",
                _helper->robot, _helper->world, pose);
        _distance_backward_mm = readRangefinder("VL53L1-backward",
                _helper->robot, _helper->world, pose);

        // Log data to file
        const int distances[] = {_distance_forward_mm, _distance_backward_mm};
        _helper->write_to_log(pose, distances, 2);
    }
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete _helper;
}

DLLEXPORT void webots_physics_init() 
{
    srand(time(NULL)); 

    _helper = new AutopilotHelper("pingpong");
}
