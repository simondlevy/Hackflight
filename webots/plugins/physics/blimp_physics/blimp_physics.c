/****************************************************************************

blimp_physics -- A blimp physics model for Webots.

Copyright (C) 2006 Laboratory of Intelligent Systems, EPFL, Lausanne
Authors:    Alexis Guanella            guanella@ini.phys.ethz.ch
            Antoine Beyeler            antoine.beyeler@epfl.ch
            Jean-Christophe Zufferey   jean-christophe.zufferey@epfl.ch
            Dario Floreano             dario.floreano@epfl.ch
Web: http://lis.epfl.ch

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

        Zufferey, J.C., Guanella, A., Beyeler, A., Floreano, D. (2006) Flying over
        the Reality Gap: From Simulated to Real Indoor Airships. Autonomous Robots,
        Springer US.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/
/*------------------------------------------------------------------------------

Author:		Antoine Beyeler (ab)

------------------------------------------------------------------------------*/

#include <plugins/physics.h>

#include "blimp2b.h"
#include "blimpmodel.h"
#include "utils.h"

// constants
const char kRobotName[] = "blimp_lis";

// globals
static dBodyID gRobotBody = NULL;

static dReal x = -3.6;
static dReal y = -1.3;
static dReal z = 1.2;
static dReal inc = 1;


DLLEXPORT void webots_physics_init() 
{
  // init global variables
  gRobotBody = dWebotsGetBodyFromDEF(kRobotName);

  if (gRobotBody == NULL)
    dWebotsConsolePrintf("!!! blimp_physics :: webots_physics_init :: error : could not get body of robot.\r\n");
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
    
    z += inc * .01;
    
    if (z > 2) { 
        inc = -1;
    }
    if (z < 1) {
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