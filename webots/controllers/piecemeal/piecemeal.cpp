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

#include <oldpids/pitchroll.hpp>
#include <oldpids/yaw.hpp>

static const float DT = 0.01;

static const float INITIAL_ALTITUDE_TARGET = 0.2;

static const float CLIMB_RATE_SCALE = 0.01;

static const float YAW_ANGLE_MAX = 200;

static const float PITCH_ROLL_DEMAND_POST_SCALE = 30; // deg

static const float YAW_PRESCALE = 160; // deg/sec

static const float THRUST_BASE = 55.385;

static const float THROTTLE_DOWN = 0.06;

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    //hf::AnglePid _anglePid = {};

    hf::YawPid _yawPid = {};

    hf::AltitudePid _altitudePid = {};

    FILE * logfp = fopen("altitude.csv", "w");
    fprintf(logfp, "time,setpoint,z,dz,output\n");

    float z_target = INITIAL_ALTITUDE_TARGET;

    while (true) {

        if (!sim.step()) {
            break;
        }

        z_target += CLIMB_RATE_SCALE * sim.throttle();

        float thrustDemand = 0;

        if (sim.hitTakeoffButton()) {

            const auto thrustOffset = _altitudePid.run(
                        DT, z_target, sim.z(), sim.dz());

            thrustDemand = THRUST_BASE + thrustOffset;

            fprintf(logfp, "%f,%f,%f,%f,%f\n",
                    sim.time(), z_target, sim.z(), sim.dz(), thrustOffset);

            fflush(logfp);
        }

        const auto resetPids = sim.throttle() < THROTTLE_DOWN;

        float rollDemand = 0;

        float pitchDemand = 0;

        const auto yawDemand =
            _yawPid.run(DT, resetPids, sim.yaw() * YAW_PRESCALE, sim.dpsi());

        /*
        hf::PositionPid::run(sim.roll(), sim.pitch(), sim.dx(), sim.dy(),
                rollDemand, pitchDemand);

        auto yawDemand = sim.yaw() * YAW_DEMAND_PRE_SCALE;

        _anglePid.run(DT,
                1.0, // fake throttle to max for now, so no PID integral reset
                rollDemand, pitchDemand, yawDemand,
                sim.phi(), sim.theta(), sim.dphi(),
                sim.dtheta(), sim.dpsi(),
                rollDemand, pitchDemand, yawDemand);

        rollDemand *= PITCH_ROLL_DEMAND_POST_SCALE;

        pitchDemand *= PITCH_ROLL_DEMAND_POST_SCALE;
        */


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
