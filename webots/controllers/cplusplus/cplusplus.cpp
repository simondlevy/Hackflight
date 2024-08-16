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

    FILE * logfp = fopen("roll.csv", "w");
    
    fprintf(logfp, 
            "state.dy,"
            "state.phi,"
            "state.dphi,"
            "stickDemands.roll,"
            "rollAngleDemand,"
            "rollAngleDemand,"
            "rollFinalDemand\n");

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

        hf::demands_t demands = { 
            stickDemands.thrust,
            stickDemands.roll,
            stickDemands.pitch,
            stickDemands.yaw
        };

        demands.thrust = 
            completedTakeoff ? 
            THRUST_BASE + K_CLIMBRATE *  (demands.thrust - state.dz) :
            hitTakeoffButton ? 
            THRUST_TAKEOFF :
            0;

        demands.roll = K_POSITION * (demands.roll - state.dy);

        const auto rollAngleDemand = demands.roll;

        demands.roll = K_PITCH_ROLL_ANGLE * (demands.roll - state.phi);

        const auto rollRateDemand = demands.roll;

        demands.roll = K_PITCH_ROLL_RATE * (demands.roll - state.dphi);

        const auto rollFinalDemand = demands.roll;

        fprintf(logfp,"%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f,%+3.3f\n",
                state.dy,
                state.phi, 
                state.dphi,
                stickDemands.roll, 
                rollAngleDemand, 
                rollRateDemand,
                rollRateDemand);

        fflush(logfp);

        demands.pitch = K_POSITION * (demands.pitch - state.dx);
        demands.pitch = K_PITCH_ROLL_ANGLE * (demands.pitch - state.theta);
        demands.pitch = K_PITCH_ROLL_RATE * (demands.pitch - state.dtheta);

        demands.yaw = K_YAW_ANGLE * (_yaw_angle_target - state.psi);
        demands.yaw = K_YAW_RATE *( demands.yaw - state.dpsi);

        hf::Mixer::runCF(demands, motors);

        sim.setMotors(motors.m1, motors.m2, motors.m3, motors.m4);
    }

    sim.close();

    return 0;
}
