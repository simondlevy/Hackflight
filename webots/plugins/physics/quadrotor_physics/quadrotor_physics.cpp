/* 
   Custom physics plugin for Hackflight simulator

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

#include <plugins/physics.h>

#include "dynamics.hpp"

static const uint32_t DYNAMICS_FREQ = 10000;

// constants
const char kRobotName[] = "quadrotor";

// globals
static dBodyID gRobotBody = NULL;

static dReal z = 0.015;
static dReal inc = 1;

static hf::Dynamics::vehicle_params_t tinyquad_params = {

    // Estimated
    4.0e-5, // force constant B [F=b*w^2]
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

static auto dynamics = hf::Dynamics(tinyquad_params, 1.f/DYNAMICS_FREQ);

DLLEXPORT void webots_physics_init() 
{
    // init global variables
    gRobotBody = dWebotsGetBodyFromDEF(kRobotName);

    if (gRobotBody == NULL) {

        dWebotsConsolePrintf("!!! quadrotor_physics :: webots_physics_init :: ");
        dWebotsConsolePrintf("error : could not get body of robot.\r\n");
    }
    else {

        dBodySetGravityMode(gRobotBody, 0);
    }
}

DLLEXPORT void webots_physics_step() 
{
    // we return if we have no robot to actuate.
    if (gRobotBody == NULL) {
        return;
    }

    dBodySetPosition(gRobotBody, 0, 0, z);

    z += inc * .005;

    if (z > 0.2) { 
        inc = -1;
    }
    if (z < 0.015) {
        inc = +1;
    }

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
