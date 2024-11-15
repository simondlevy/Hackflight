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

#include "quadrotor2b.h"
#include "quadrotormodel.h"
#include "utils.h"

// constants
const char kRobotName[] = "quadrotor";

// globals
static dBodyID gRobotBody = NULL;

static dReal x = 0;
static dReal y = 0;
static dReal z = 0.015;
static dReal inc = 1;

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
    
    dBodySetPosition(gRobotBody, x, y, z);
    
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
  return 0;
}

DLLEXPORT void webots_physics_cleanup() 
{
}
