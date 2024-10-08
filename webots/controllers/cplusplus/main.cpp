/* C++ flight simulator support for Hackflight Copyright (C) 2024 Simon D. Levy

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

#include <pids/altitude1.hpp>
#include <pids/altitude2.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static const float DT = 0.01;

static const float INITIAL_ALTITUDE_TARGET = 0.2;

static const float CLIMB_RATE_SCALE = 0.01;

static const float YAW_PRESCALE = 160; // deg/sec

static const float THRUST_BASE = 55.385;

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_POST_SCALE = 50;

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    hf::PitchRollAnglePid pitchRollAnglePid = {};
    hf::PitchRollRatePid pitchRollRatePid = {};

    hf::YawRatePid yawRatePid = {};

    hf::AltitudePid1 altitudePid1 = {};

    float z_target = INITIAL_ALTITUDE_TARGET;

    while (true) {

        hf::demands_t demands = {};
        hf::state_t state = {};
        bool button = false;

        if (!sim.step(state, demands)) {
            break;
        }

        demands.yaw *= YAW_PRESCALE;

        const auto resetPids = demands.thrust < THROTTLE_DOWN;

        if (sim.isSpringy()) {

            if (sim.hitTakeoffButton()) {

                z_target += CLIMB_RATE_SCALE * demands.thrust;

                demands.thrust = z_target;

                altitudePid1.run(DT, state, demands);

                demands.thrust += THRUST_BASE;
            }
        }

        else {
            printf("%+3.3f\n", demands.thrust);
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
