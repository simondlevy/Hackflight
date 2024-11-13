#include <plugins/physics.h>

#include "quadrotor2b.h"
#include "quadrotormodel.h"
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
