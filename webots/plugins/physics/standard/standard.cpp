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

#include <stdio.h>

#include <hackflight.hpp>
#include <sim/dynamics.hpp>
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

// Called by webots_physics_init(); unneeded here
void setup_controllers()
{
}

hf::state_t estimate_state(const hf::Dynamics & dynamics)
{
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

hf::demands_t run_controllers(
        const float pid_dt,
        const hf::siminfo_t & siminfo,
        const hf::state_t & state,
        const hf::demands_t & open_loop_demands)
{
        // Throttle-down should reset pids
        const auto resetPids = open_loop_demands.thrust < THROTTLE_DOWN;

        // Start with open-loop demands
        hf::demands_t demands = {
            open_loop_demands.thrust,
            open_loop_demands.roll,
            open_loop_demands.pitch,
            open_loop_demands.yaw
        };

        if (siminfo.requested_takeoff) {

            _altitudePid.run(siminfo.is_springy, pid_dt, state, demands);

            demands.thrust += MOTOR_HOVER;
        }

        hf::PositionPid::run(state, demands);

        _pitchRollAnglePid.run(pid_dt, resetPids, state, demands);

        _pitchRollRatePid.run(pid_dt, resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        _yawRatePid.run(pid_dt, resetPids, state, demands);

        return demands;
}
    

