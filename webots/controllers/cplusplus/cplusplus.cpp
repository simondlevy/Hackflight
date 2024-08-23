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

#include <sim.hpp>

#include <hackflight.hpp>
#include <mixers.hpp>

#include <pids/angle.hpp>
#include <pids/climbrate.hpp>

static const float DT = 0.01;

static const float K_PITCH_ROLL_ANGLE = 6;
static const float K_PITCH_ROLL_RATE = 0.0125;

static const float K_CLIMBRATE = 25;

static const float K_POSITION = 10;

static const float YAW_ANGLE_MAX = 200;

static const float PITCH_ROLL_DEMAND_POST_SCALE = 30; // deg

static const float YAW_DEMAND_PRE_SCALE = 160; // deg/sec

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    hf::AnglePid _anglePid = {};

    while (true) {

        if (!sim.step()) {
            break;
        }

        hf::quad_motors_t motors = {};

        const auto thrust_demand = hf::ClimbRatePid::run(
                sim.hitTakeoffButton(),
                sim.completedTakeoff(),
                sim.throttle(),
                sim.dz());

        /*
        const auto thrust_demand = 
            sim.completedTakeoff() ? 
            THRUST_BASE + K_CLIMBRATE *  (sim.throttle() - sim.dz()) :
            sim.hitTakeoffButton() ? 
            THRUST_TAKEOFF :
            0;*/

        auto roll_demand = K_POSITION * (sim.roll() - sim.dy());

        auto pitch_demand = K_POSITION * (sim.pitch() - sim.dx());

        auto yaw_demand = sim.yaw() * YAW_DEMAND_PRE_SCALE;

        _anglePid.run(DT,
                1.0, // fake throttle to max for now, so no PID integral reset
                roll_demand,
                pitch_demand,
                yaw_demand,
                sim.phi(),
                sim.theta(),
                sim.dphi(),
                sim.dtheta(),
                sim.dpsi(),
                roll_demand,
                pitch_demand,
                yaw_demand);

        roll_demand *= PITCH_ROLL_DEMAND_POST_SCALE;

        pitch_demand *= PITCH_ROLL_DEMAND_POST_SCALE;

        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runBetaFlightQuadX(
                thrust_demand, roll_demand, pitch_demand, yaw_demand,
                m1, m2, m3, m4);

        sim.setMotors(m1, m2, m3, m4);
    }

    sim.close();

    return 0;
}
