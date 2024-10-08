/* 
   C++ flight simulator support for Hackflight Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.hpp>
#include <mixers.hpp>
#include <sim.hpp>

#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static const float DT = 0.01;

static const float YAW_PRESCALE = 160; // deg/sec

static const float THRUST_BASE = 55.385;

static const float THROTTLE_DOWN = 0.06;

static const float THROTTLE_DEADBAND = 0.2;

static const float PITCH_ROLL_POST_SCALE = 50;

// For springy-throttle gamepads / keyboard
static const float INITIAL_ALTITUDE_TARGET = 0.2;
static const float CLIMB_RATE_SCALE = 0.01;

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    hf::PitchRollAnglePid pitchRollAnglePid = {};
    hf::PitchRollRatePid pitchRollRatePid = {};

    hf::YawRatePid yawRatePid = {};

    hf::AltitudePid altitudePid = {};

    bool requestedTakeoff = false;

    // This initial value will be ignored for traditional (non-springy)
    // throttle
    float z_target = INITIAL_ALTITUDE_TARGET;

    while (true) {

        if (!sim.step()) {
            break;
        }

        auto demands = sim.getDemands();

        const auto state = sim.getState();

        demands.yaw *= YAW_PRESCALE;

        const auto resetPids = demands.thrust < THROTTLE_DOWN;

        // Throttle control begins when once takeoff is requested, either by
        // hitting a button or key ("springy", self-centering throttle) or by
        // raising the non-self-centering throttle stick
        if (sim.requestedTakeoff()) {

            // "Springy" (self-centering) throttle or keyboard: accumulate 
            // altitude target based on stick deflection, and attempt
            // to maintain target via PID control
            if (sim.isSpringy()) {

                z_target += CLIMB_RATE_SCALE * demands.thrust;
                demands.thrust = z_target;
                altitudePid.run(DT, state, demands);
                demands.thrust += THRUST_BASE;
            }

            // Traditional (non-self-centering) throttle: 
            //
            //   (1) In throttle deadband (mid position), fix an altitude target
            //       and attempt to maintain it via PID control
            //
            //   (2) Outside throttle deadband, get thrust from stick deflection
            else {

                static bool _was_in_deadband;
                const auto in_deadband = fabs(demands.thrust) < THROTTLE_DEADBAND;
                z_target = in_deadband && !_was_in_deadband ? state.z : z_target;
                _was_in_deadband = in_deadband;
                if (in_deadband) {
                    demands.thrust = z_target;
                    altitudePid.run(DT, state, demands);
                }
                demands.thrust += THRUST_BASE;
            }

        }

        hf::PositionPid::run(state, demands);

        pitchRollAnglePid.run(DT, resetPids, state, demands);

        pitchRollRatePid.run(DT, resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        yawRatePid.run(DT, resetPids, state, demands);

        hf::quad_motors_t motors= {};

        hf::Mixer::runBetaFlightQuadX(demands, motors);

        sim.setMotors(motors);
    }

    sim.close();

    return 0;
}
