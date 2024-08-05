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

#include <newpids/altitude.hpp>
#include <newpids/pitch_roll_angle.hpp>
#include <newpids/pitch_roll_rate.hpp>
#include <newpids/position.hpp>
#include <newpids/yaw.hpp>

static const float PITCH_ROLL_ANGLE_KP = 6e0;

static const float PITCH_ROLL_RATE_KP = 1.25e-2;

static const float YAW_RATE_KP = 1.20e-2;

static const float YAW_ANGLE_MAX = 200;

// Motor thrust constants
static const float TBASE = 56;
static const float TSCALE = 0.25;
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

int main(int argc, char ** argv)
{
    hf::PositionController positionController= {};
    hf::PitchRollAngleController pitchRollAngleController= {};
    hf::PitchRollRateController pitchRollRateController= {};
    hf::AltitudeController altitudeController= {};
    hf::YawController yawController= {};

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

        positionController.run(state, demands);

        pitchRollAngleController.run(PITCH_ROLL_ANGLE_KP, state, demands);

        pitchRollRateController.run(PITCH_ROLL_RATE_KP, state, demands);

        altitudeController.run(state, _altitude_target, demands);

        yawController.run(YAW_RATE_KP, state, _yaw_angle_target, demands);

        demands.thrust = landed ? TMIN : TBASE + TSCALE * demands.thrust;
        
        hf::Mixer::runCF(demands, motors);

        sim.setMotors(motors.m1, motors.m2, motors.m3, motors.m4);
    }

    sim.close();

    return 0;
}
