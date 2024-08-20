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

static const float K_PITCH_ROLL_ANGLE = 6;
static const float K_PITCH_ROLL_RATE = 0.0125;

static const float K_YAW_ANGLE = 6;
static const float K_YAW_RATE = 0.012;

static const float K_CLIMBRATE = 25;

static const float K_POSITION = 25;

static const float YAW_ANGLE_MAX = 200;

static const float YAW_DEMAND_SCALE = .01;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

static float cap_yaw_angle(const float angle)
{
    const float angle1 = angle > 180 ? angle - 360 : angle;

    return angle1 < -180 ? angle1 + 360 : angle1;
}

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    while (true) {

        hf::state_t state = {};

        hf::demands_t stickDemands = {};

        bool hitTakeoffButton = false;

        bool completedTakeoff = false;

        if (!sim.step(stickDemands, state, hitTakeoffButton, completedTakeoff)) {
            break;
        }

        static float _yaw_angle_target;

        hf::quad_motors_t motors = {};

        _yaw_angle_target = cap_yaw_angle(_yaw_angle_target + 
                YAW_ANGLE_MAX * stickDemands.yaw * YAW_DEMAND_SCALE);

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

        auto yaw_demand = K_YAW_ANGLE * (_yaw_angle_target - state.psi);
        printf("%+3.3f\n", yaw_demand);
        yaw_demand = K_YAW_RATE *( yaw_demand - state.dpsi);

        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runCF(
                thrust_demand, roll_demand, pitch_demand, yaw_demand,
                m1, m2, m3, m4);

        sim.setMotors(m1, m2, m3, m4);
    }

    sim.close();

    return 0;
}
