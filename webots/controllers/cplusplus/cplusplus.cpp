/*
   C++ flight simulator support for Hackflight

   Copyright (C) 2024 Simon D. Levy

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

#include <webots.hpp>

#include <hackflight.hpp>
#include <mixers.hpp>
#include <utils.hpp>

static const float K_PITCH_ROLL_ANGLE = 6;
static const float K_PITCH_ROLL_RATE = 0.0125;

static const float K_YAW_ANGLE = 6;
static const float K_YAW_RATE = 0.012;

static const float K_ALTITUDE = 2;
static const float K_CLIMBRATE = 25;

static const float K_POSITION = 25;

static const float YAW_ANGLE_MAX = 200;

// Motor thrust constants
static const float TBASE = 56;
static const float TMIN = 0;

static const float INITIAL_ALTITUDE_TARGET = 0.2;

// We consider throttle inputs above this below this value to be
// positive for takeoff
static constexpr float THROTTLE_ZERO = 0.05;

static constexpr float THROTTLE_SCALE = 0.005;

// We consider altitudes below this value to be the ground
static constexpr float ZGROUND = 0.05;

static constexpr float YAW_DEMAND_SCALE = .01;

typedef enum {

    STATUS_LANDED,
    STATUS_TAKING_OFF,
    STATUS_FLYING

} flyingStatus_e;

static float cap_yaw_angle(const float angle)
{
    const float angle1 = angle > 180 ? angle - 360 : angle;

    return angle1 < -180 ? angle1 + 360 : angle1;
}

static float control(const float k, const float target, const float actual)
{
    return k * (target - actual);
}

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init();

    while (true) {

        hf::state_t state = {};

        hf::demands_t stickDemands = {};

        if (!sim.step(stickDemands, state)) {
            break;
        }

        static flyingStatus_e _status;

        static float _altitude_target;

        static float _yaw_angle_target;

        hf::quad_motors_t motors = {};

        _altitude_target =
            _status == STATUS_FLYING ? 
            _altitude_target + THROTTLE_SCALE * stickDemands.thrust :
            _status == STATUS_LANDED ?
            INITIAL_ALTITUDE_TARGET :
            _altitude_target;

        _yaw_angle_target = cap_yaw_angle(_yaw_angle_target + 
                YAW_ANGLE_MAX * stickDemands.yaw * YAW_DEMAND_SCALE);

        _status = 

            _status == STATUS_TAKING_OFF  && state.z > ZGROUND ?  
            STATUS_FLYING :

            _status == STATUS_FLYING && state.z <= ZGROUND ?  
            STATUS_LANDED :

            _status == STATUS_LANDED && 
            stickDemands.thrust > THROTTLE_ZERO ? 
            STATUS_TAKING_OFF :

            _status;

        const auto landed = _status == STATUS_LANDED;

        hf::demands_t demands = { 
            stickDemands.thrust,
            stickDemands.roll,
            stickDemands.pitch,
            stickDemands.yaw
        };

        const auto dz_target = control(K_ALTITUDE, _altitude_target, state.z);

        const auto thrust = control(K_CLIMBRATE,  dz_target, state.dz);

        demands.roll = control(K_POSITION, demands.roll, state.dy);

        demands.roll = control(K_PITCH_ROLL_ANGLE, demands.roll, state.phi);

        demands.roll = control(K_PITCH_ROLL_RATE, demands.roll, state.dphi);

        demands.pitch = control(K_POSITION, demands.pitch, state.dx);

        demands.pitch = control(
                K_PITCH_ROLL_ANGLE, demands.pitch, state.theta);

        demands.pitch = control(
                K_PITCH_ROLL_RATE, demands.pitch, state.dtheta);

        demands.yaw = control(K_YAW_ANGLE, _yaw_angle_target, state.psi);

        demands.yaw = control(K_YAW_RATE, demands.yaw, state.dpsi);

        demands.thrust = landed ? TMIN : TBASE + thrust;
        
        hf::Mixer::runCF(demands, motors);

        sim.setMotors(motors.m1, motors.m2, motors.m3, motors.m4);
    }

    sim.close();

    return 0;
}
