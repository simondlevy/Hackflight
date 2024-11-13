#pragma once

//----------------------------------------------------------------------------//
// Includes
#include <ode/common.h>

//----------------------------------------------------------------------------//
// Dynamic model parameters
static const dReal g = REAL(9.81);  // same value in Webots

//----------------------------------------------------------------------------//
// Dynamic model function

void bmod_ComputeGenForces(const dReal *inGenPosA, const dReal *inGenVelAB, const dReal *inPropThrusts, dReal *outGenForceAB);
