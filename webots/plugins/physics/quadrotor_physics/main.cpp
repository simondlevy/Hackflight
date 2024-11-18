/* 
 *   Custom physics plugin for Hackflight simulator
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// Webots
#include <plugins/physics.h>

// Hackflight
#include <hackflight.hpp>
#include <pids/altitude.hpp>
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>
#include <mixers/bfquadx.hpp>
#include <utils.hpp>

#include "dynamics.hpp"

static const uint32_t DYNAMICS_FREQ = 1e5; // Hz

static const uint32_t PID_FREQ = 1e3; // Hz

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_POST_SCALE = 50;

// XXX can we get this automatically?
static const double ROBOT_TIMESTEP_MSEC = 32;

static const char ROBOT_NAME[] = "quadrotor";

static dBodyID _robotBody;

static hf::AltitudePid _altitudePid;

static hf::PitchRollAnglePid _pitchRollAnglePid;

static hf::PitchRollRatePid _pitchRollRatePid;

static hf::YawRatePid _yawRatePid;

static hf::Dynamics::vehicle_params_t tinyquad_params = {

    1.8e-5, // force constant B [F=b*w^2]
    3.1e-2, // arm length L [m]

    2.0e0 , // torque constant D [T=d*w^2]
    0.050,  // mass M [kg]
    2.0e0,  // Ix [kg*m^2]
    2.0e0,  // Iy [kg*m^2]
    3.0e0,  // Iz [kg*m^2]
    3.8e-3  // Jr prop inertial [kg*m^2]
};

static hf::demands_t getOpenLoopDemands()
{
    static hf::demands_t _demands;

    int size = 0;

    const auto buffer = (hf::demands_t *)dWebotsReceive(&size);

    if (size == sizeof(hf::demands_t)) {
        memcpy(&_demands, buffer, sizeof(hf::demands_t));
    }

    return _demands;
}

static auto dynamics = hf::Dynamics(tinyquad_params, 1./DYNAMICS_FREQ);

DLLEXPORT void webots_physics_init() 
{
    // init global variables
    _robotBody = dWebotsGetBodyFromDEF(ROBOT_NAME);

    if (_robotBody == NULL) {

        dWebotsConsolePrintf("quadrotor_physics :: webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(_robotBody, 0);
    }
}

DLLEXPORT void webots_physics_step() 
{
    // we return if we have no robot to actuate.
    if (_robotBody == NULL) {
        return;
    }

    const auto open_loop_demands = getOpenLoopDemands();

    // Run control in outer loop
    for (uint32_t j=0; j< ROBOT_TIMESTEP_MSEC * PID_FREQ / 1000; ++j) {

        // Get vehicle state
        const auto state = dynamics.getState();

        dWebotsConsolePrintf("new: %+3.3f\n", state.dy);

        // Start with open-loop demands
        hf::demands_t demands = {
            open_loop_demands.thrust,
            open_loop_demands.roll,
            0,
            open_loop_demands.yaw
        };

        // Run PID controllers to get final demands

        const auto springyThrottle = true; // XXX
        const auto resetPids = open_loop_demands.thrust < THROTTLE_DOWN;
        static const float pid_dt  = 1. / PID_FREQ;

        _altitudePid.run(springyThrottle, pid_dt, state, demands);

        hf::PositionPid::run(state, demands);

        _pitchRollAnglePid.run(pid_dt, resetPids, state, demands);

        _pitchRollRatePid.run(pid_dt, resetPids, state, demands,
                PITCH_ROLL_POST_SCALE);

        _yawRatePid.run(pid_dt, resetPids, state, demands);

        // Run mixer to get motors spins from demands
        hf::BfQuadXMixer mixer = {};
        float motors[4] = {};
        mixer.run(demands, motors);

        //printf("m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f\n\n",
        //       motors[0], motors[1], motors[2], motors[3]);

        // Run dynamics in inner loop to update state with motors
        for (uint32_t k=0; k<DYNAMICS_FREQ / PID_FREQ; ++k) {
            dynamics.update(motors, &mixer);
        }
    }

    // Get current state from dynamics
    const auto state = dynamics.getState();

    // Convert Euler angles to radians, negating psi for nose-right positive
    const hf::axis3_t euler = {
        hf::Utils::DEG2RAD * state.phi, 
        hf::Utils::DEG2RAD * state.theta,
        -hf::Utils::DEG2RAD * state.psi};

    // Turn Euler angles into quaternion
    hf::axis4_t quat = {};
    hf::Utils::euler2quat(euler, quat);
    const dQuaternion q = {quat.w, quat.x, quat.y, quat.z};

    printf("y=%+3.3f  dy=%+3.3f\n", state.y, state.dy);

    // Set robot posed based on state, negating for rightward negative
    dBodySetPosition(_robotBody, state.x, -state.y, state.z);
    dBodySetQuaternion(_robotBody, q);
}

DLLEXPORT int webots_physics_collide(dGeomID g1, dGeomID g2) 
{
    (void)g1;
    (void)g2;

    return 0;
}

DLLEXPORT void webots_physics_cleanup() 
{
}
