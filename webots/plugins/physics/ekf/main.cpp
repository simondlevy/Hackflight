/* 
 * Custom physics plugin for Hackflight simulator using C++ PID controllers
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

#include "../support.hpp"

#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_POST_SCALE = 50;

static const float MOTOR_HOVER = 74.565; // rad/sec

static hf::AltitudePid _altitudePid;

static hf::PitchRollAnglePid _pitchRollAnglePid;

static hf::PitchRollRatePid _pitchRollRatePid;

static hf::YawRatePid _yawRatePid;

static hf::state_t estimateState()
{
    // For now we run the state estimator at the same rate as the control loop
    static auto dt = 1 / (float)PID_FREQ;

    // Get simulated gyrometer values
    const auto gyro = hf::Gyrometer::read(dynamics);

     // Get simulated accelerometer values
    const auto accel = hf::Accelerometer::read(dynamics);

   // Get simulated rangefinder distance
    const auto range = hf::Rangefinder::read(dynamics);

    // Get simulated optical flow
    const auto flow = hf::OpticalFlow::read(dynamics);

    (void)accel;
    (void)gyro;
    (void)flow;
    (void)range;
    (void)dt;
    (void)flow;

    // XXX Cheat and use ground-truth state for now
    return hf::state_t {
        dynamics._x1,
            dynamics._x2 * cos(dynamics._x11) -
                dynamics._x4 * sin(dynamics._x11),
        dynamics._x3,
        -(dynamics._x2 * sin(dynamics._x11) +
                    dynamics._x4 * cos(dynamics._x11)),
        dynamics._x5,
        dynamics._x6,
            hf::Utils::RAD2DEG* dynamics._x7,
            hf::Utils::RAD2DEG* dynamics._x8,
            hf::Utils::RAD2DEG* dynamics._x9,
            hf::Utils::RAD2DEG* dynamics._x10,
            hf::Utils::RAD2DEG* dynamics._x11,
            hf::Utils::RAD2DEG* dynamics._x12,
    };
}


// This is called by Webots in the outer (display, kinematics) loop
DLLEXPORT void webots_physics_step() 
{
    hf::siminfo_t siminfo = {};

    if (!getSimInfo(siminfo)) {
        return;
    }

    // Run control in middle loop
    for (uint32_t j=0; j <outerLoopCount(siminfo);  ++j) {
        
        // Start with open-loop demands
        hf::demands_t demands = {
            siminfo.demands.thrust,
            siminfo.demands.roll,
            siminfo.demands.pitch,
            siminfo.demands.yaw
        };

        // Throttle-down should reset pids
        const auto resetPids = demands.thrust < THROTTLE_DOWN;

        const auto state = estimateState();

        // Run PID controllers to get final demands
        
        if (siminfo.requested_takeoff) {

            _altitudePid.run(siminfo.is_springy, pidDt(), state, demands);

            demands.thrust += MOTOR_HOVER;
        }

        hf::PositionPid::run(state, demands);

        _pitchRollAnglePid.run(pidDt(), resetPids, state, demands);

        _pitchRollRatePid.run(pidDt(), resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        _yawRatePid.run(pidDt(), resetPids, state, demands);

        // Update dynamics in innermost loop
        updateDynamics(demands);
    }

    // Set pose in outermost loop
    setPose(dynamics);
}
    
// Called by webots_physics_init(); unneeded here
void setup_controllers()
{
}
