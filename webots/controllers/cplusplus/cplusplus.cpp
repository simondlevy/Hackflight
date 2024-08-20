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

static const float DT = 0.01;

static const float K_PITCH_ROLL_ANGLE = 6;
static const float K_PITCH_ROLL_RATE = 0.0125;

static const float K_CLIMBRATE = 25;

static const float K_POSITION = 25;

static const float YAW_ANGLE_MAX = 200;

static const float YAW_DEMAND_SCALE = 160; // deg/sec

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    hf::AnglePid _anglePid = {};

    while (true) {

        hf::state_t state = {};

        hf::demands_t stickDemands = {};

        bool hitTakeoffButton = false;

        bool completedTakeoff = false;

        if (!sim.step(stickDemands, state, hitTakeoffButton, completedTakeoff)) {
            break;
        }

        hf::quad_motors_t motors = {};

        const auto thrust_demand = 
            completedTakeoff ? 
            THRUST_BASE + K_CLIMBRATE *  (stickDemands.thrust - state.dz) :
            hitTakeoffButton ? 
            THRUST_TAKEOFF :
            0;

        auto roll_demand = K_POSITION * (stickDemands.roll - state.dy);

        roll_demand = K_PITCH_ROLL_ANGLE * (roll_demand - state.phi);
        roll_demand = K_PITCH_ROLL_RATE * (roll_demand - state.dphi);

        auto pitch_demand = K_POSITION * (stickDemands.pitch - state.dx);

        pitch_demand = K_PITCH_ROLL_ANGLE * (pitch_demand - state.theta);
        pitch_demand = K_PITCH_ROLL_RATE * (pitch_demand - state.dtheta);

        auto yaw_demand = stickDemands.yaw * YAW_DEMAND_SCALE;

        float new_roll_demand = 0;
        float new_pitch_demand = 0;

        _anglePid.run(DT,
                roll_demand,
                pitch_demand,
                yaw_demand,
                state.phi,
                state.theta,
                2000,
                state.dphi,
                state.dtheta,
                state.dpsi,
                new_roll_demand,
                new_pitch_demand,
                yaw_demand);

        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runCF(
                thrust_demand, roll_demand, pitch_demand, yaw_demand,
                m1, m2, m3, m4);

        sim.setMotors(m1, m2, m3, m4);
    }

    sim.close();

    return 0;
}
