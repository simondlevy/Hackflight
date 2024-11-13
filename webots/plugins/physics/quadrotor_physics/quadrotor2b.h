#pragma once

#include <ode/common.h>

static const dReal b2b_m = REAL(0.184);  // mass 
static const dReal b2b_addedMass[] =     // added mass parameters [m]
  {REAL(0.033), REAL(0.129), REAL(0.129)};

// Linear damping terms
static const dReal b2b_LinearDamp[] = {REAL(0),  // (here only linear
                                       REAL(0),  // damping for
                                       REAL(0),  // rotation (i=3,4,5)
                                       REAL(0.005), REAL(0.0079), REAL(0.0079)};

// Quadratic damping terms
static const dReal b2b_QuadraticDamp[] = {REAL(0.07),  //(here only
                                          REAL(0.28),  // quadratic
                                          REAL(0.22),  // damping for
                                          REAL(0),     // translation
                                          REAL(0),     // (terms i= 0,1,2)
                                          REAL(0)};

static const dReal b2b_rz = REAL(0.125);  // r is vector CG - CB (buoyancy frame) [m]

//------------------------------------------------------------------------------
// Engines: command [-1.0..+1.0] to thrust [N] converters

void b2b_commandsToThrust(double inFront, double inYaw, double inVert, dReal *propThrusts);
void b2b_compThrustWrench(const dReal *propThrusts, dReal *genForceAB);
