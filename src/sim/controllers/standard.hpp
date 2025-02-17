/* 
 * Standard PID controllers for simulation
 *
 * Copyright (C) 2024 Simon D. Levy
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

#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>
#include <sim/vehicles/diyquad.hpp>

static const float THROTTLE_DOWN = 0.06;
static const float PITCH_ROLL_POST_SCALE = 50;

namespace hf {

    static AltitudePid _altitudePid;
    static PitchRollAnglePid _pitchRollAnglePid;
    static PitchRollRatePid _pitchRollRatePid;
    static YawRatePid _yawRatePid;

    void setup_controllers()
    {
    }

    demands_t run_controllers(
            const float pid_dt,
            const siminfo_t & siminfo,
            const state_t & state)
    {
        const auto open_loop_demands = siminfo.demands;

        // Throttle-down should reset pids
        const auto resetPids = open_loop_demands.thrust < THROTTLE_DOWN;

        // Start with open-loop demands
        demands_t demands = {
            open_loop_demands.thrust,
            open_loop_demands.roll,
            open_loop_demands.pitch,
            open_loop_demands.yaw
        };

        if (siminfo.requested_takeoff) {

            _altitudePid.run(siminfo.is_springy, pid_dt, state, demands);

            demands.thrust += MOTOR_HOVER;
        }

        PositionPid::run(state, demands);

        _pitchRollAnglePid.run(pid_dt, resetPids, state, demands);

        _pitchRollRatePid.run(pid_dt, resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        _yawRatePid.run(pid_dt, resetPids, state, demands);

        return demands;
    }
}
