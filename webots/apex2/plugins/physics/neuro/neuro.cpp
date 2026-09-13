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
#include <time.h>

// Hackflight
#include <sim/dynamics.hpp>
#include <plugin_helper.hpp>
#include "../autopilot.hpp"

// SimSensors
#include <simsensors/src/world.hpp>
#include <simsensors/src/robot.hpp>
#include <simsensors/src/sensors/rangefinder.hpp>

static constexpr float kSpeed = 0.5;

static AutopilotHelper * ahelper_;

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

    extern double encoder_vals[2];
    encoder_vals[0] = diff;
    encoder_vals[1] = dydt;

    extern void encode_run_decode();
    encode_run_decode();

    extern double decoder_vals[1];
    const int8_t direction = decoder_vals[0] == 1 ? +1 : -1;

    printf("%+05.0f,%+6.6f => %+1.0f\n",
            encoder_vals[0], encoder_vals[1], decoder_vals[0]);

    return hf::Setpoint(0, 0, direction * kSpeed, 0);
}

// Returns false on collision, true otherwise
// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    const auto message = PluginHelper::GetMessage();

    // Get current vehicle state
    const auto state = ahelper_->GetState(message);

    static int _distance_forward_mm;
    static int _distance_backward_mm;

    // Replace open-loop setpoint with setpoint from autopilot if
    // available
    const auto setpoint = message.mode == hf::kModeAutonomous ?
        getSetpoint(_distance_forward_mm, 
                _distance_backward_mm, state.dy) :
        message.setpoint;

    // Get vehicle pose based on setpoint
    const auto pose = ahelper_->GetPose(message.mode, setpoint);

    // Grab rangefinder readings for next iteration
    _distance_forward_mm = readRangefinder("VL53L1-forward",
            ahelper_->robot, ahelper_->world, pose);
    _distance_backward_mm = readRangefinder("VL53L1-backward",
            ahelper_->robot, ahelper_->world, pose);

    // Log data to file
    //const int distances[] = {_distance_forward_mm, _distance_backward_mm};
    //ahelper_->WriteToLog(pose, distances, 2);
}

DLLEXPORT void webots_physics_cleanup() 
{
    delete ahelper_;
}

DLLEXPORT void webots_physics_init() 
{
    srand(time(NULL)); 

    ahelper_ = new AutopilotHelper("pingpong");
}
