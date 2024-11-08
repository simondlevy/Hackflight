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
#include <madgwick.hpp>
#include <mixers/bfquadx.hpp>
#include <sim.hpp>
#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

static const float PITCH_ROLL_POST_SCALE = 50;

static const float DT_PID = 0.01;

static const float DT_ESTIMATOR = 0.033;

static const float THROTTLE_DOWN = 0.06;

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    hf::PitchRollAnglePid pitchRollAnglePid = {};
    hf::PitchRollRatePid pitchRollRatePid = {};

    hf::YawRatePid yawRatePid = {};

    hf::AltitudePid altitudePid = {};

    hf::BfQuadXMixer mixer = {};

    hf::Madgwick madgwick = {};

    madgwick.initialize();

    while (true) {

        if (!sim.step()) {
            break;
        }

        auto demands = sim.getDemands();

        const auto gyro = sim.readGyro();

        const auto accel = sim.readAccel();

        hf::state_t state = { };

        sim.getMiniState(state.dx, state.dy, state.z, state.dz);

        madgwick.getAngles(
                DT_ESTIMATOR, gyro, accel, state.phi, state.theta, state.psi);

        // Get angular velocities directly from gyro
        state.dphi = gyro.x;
        state.dtheta = -gyro.y;
        state.dpsi = gyro.z;

        const auto resetPids = demands.thrust < THROTTLE_DOWN;

        // Throttle control begins when once takeoff is requested, either by
        // hitting a button or key ("springy", self-centering throttle) or by
        // raising the non-self-centering throttle stick
        if (sim.requestedTakeoff()) {

            altitudePid.run(sim.isSpringy(), DT_PID, state, demands);

            demands.thrust += hf::Simulator::MOTOR_HOVER;
        }

        hf::PositionPid::run(state, demands);

        pitchRollAnglePid.run(DT_PID, resetPids, state, demands);

        pitchRollRatePid.run(DT_PID, resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        yawRatePid.run(DT_PID, resetPids, state, demands);

        float motors[4] = {};

        mixer.run(demands, motors);

        sim.setMotors(motors);
    }

    sim.close();

    return 0;
}
