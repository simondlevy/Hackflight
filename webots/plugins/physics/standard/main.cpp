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

        //hf::PositionPid::run(state, demands);

        demands.roll *= 30;
        demands.pitch *= 30;

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
