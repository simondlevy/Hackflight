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
#include <pids/yaw_rate.hpp>
#include <mixers/bfquadx.hpp>

#include "dynamics.hpp"

static const uint32_t DYNAMICS_FREQ = 100000; // Hz

static const uint32_t PID_FREQ = 1000; // Hz

static const float MOTOR_HOVER = 55.385; // rad/sec

// XXX can we get this automatically?
static const double ROBOT_TIMESTEP = 32;

static const char ROBOT_NAME[] = "quadrotor";

static dBodyID _robotBody;

static hf::AltitudePid _altitudePid;

static hf::YawRatePid _yawRatePid;

static hf::Dynamics::vehicle_params_t tinyquad_params = {

    // Estimated
    1.8e-5, // force constant B [F=b*w^2]
    4.0e0, // torque constant D [T=d*w^2]

    // These agree with values in .proto file
    0.050,  // mass M [kg]
    0.031,  // arm length L [m]

    // Estimated
    2,      // Ix [kg*m^2]
    2,      // Iy [kg*m^2]
    3,      // Iz [kg*m^2]
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
    for (uint32_t j=0; j< ROBOT_TIMESTEP * PID_FREQ / 1000; ++j) {

        // Get vehicle state
        const auto state = dynamics.getState();

        // Start with open-loop demands
        hf::demands_t demands = {
            open_loop_demands.thrust,
            0,
            0,
            open_loop_demands.yaw
        };

        // Run PID controllers to get final demands

        static const float pid_dt  = 1. / PID_FREQ;

        _altitudePid.run(true, pid_dt, state, demands);

        _yawRatePid.run(pid_dt, false, state, demands);

        demands.thrust += MOTOR_HOVER;

        // Run mixer to get motors spins from demands
        hf::BfQuadXMixer mixer = {};

        float motors[4] = {};

        mixer.run(demands, motors);

        // Run dynamics in inner loop
        for (uint32_t k=0; k<DYNAMICS_FREQ / PID_FREQ; ++k) {

            dynamics.update(motors);
        }
    }

    const auto state = dynamics.getState();

    dBodySetPosition(_robotBody, 0, 0, state.z);

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
