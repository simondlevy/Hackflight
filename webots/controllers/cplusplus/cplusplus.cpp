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

#include <pids/altitude.hpp>
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

static const float PITCH_ROLL_POST_SCALE = 1e-4;

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    hf::PitchRollAnglePid pitchRollAnglePid = {};
    hf::PitchRollRatePid pitchRollRatePid = {};

    hf::YawRatePid yawRatePid = {};

    hf::AltitudePid altitudePid = {};

    FILE * logfp = fopen("roll.csv", "w");
    fprintf(logfp, "time,roll_stick,dy,roll_angle_demand\n");

    float z_target = INITIAL_ALTITUDE_TARGET;

    while (true) {

        if (!sim.step()) {
            break;
        }

        z_target += CLIMB_RATE_SCALE * sim.throttle();

        float thrustDemand = 0;

        const auto resetPids = sim.throttle() < THROTTLE_DOWN;

        float rollDemand = 0;
        float pitchDemand = 0;

        if (sim.hitTakeoffButton()) {

            const auto thrustOffset = altitudePid.run(
                        DT, z_target, sim.z(), sim.dz());

            thrustDemand = THRUST_BASE + thrustOffset;

        }

        hf::PositionPid::run(sim.roll(), sim.pitch(), sim.dx(), sim.dy(),
                rollDemand, pitchDemand);

        if (sim.hitTakeoffButton()) {

            fprintf(logfp, "%f,%f,%f,%f,%f\n",
                    sim.time(), sim.roll(), sim.dy(), rollDemand, sim.phi());

            fflush(logfp);
        }

        pitchRollAnglePid.run(DT, resetPids, rollDemand, pitchDemand,
                sim.phi(), sim.theta(), rollDemand, pitchDemand);

        pitchRollRatePid.run( DT, resetPids, rollDemand, pitchDemand,
                sim.dphi(), sim.dtheta(), rollDemand, pitchDemand);

        rollDemand *= PITCH_ROLL_POST_SCALE;

        pitchDemand *= PITCH_ROLL_POST_SCALE;

        const auto yawDemand =
            yawRatePid.run(DT, resetPids, sim.yaw() * YAW_PRESCALE, sim.dpsi());

        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runBetaFlightQuadX(
                thrustDemand,
                rollDemand,
                pitchDemand,
                yawDemand,
                m1, m2, m3, m4);

        sim.setMotors(m1, m2, m3, m4);
    }

    sim.close();

    return 0;
}
