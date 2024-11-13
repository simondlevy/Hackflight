#include <stdio.h>
#include <string.h>

#include "quadrotor2b.h"
#include "quadrotormodel.h"
#include "utils.h"

//----------------------------------------------------------------------------//
// Prototypes

// computes damping wrench
void compDampingWrench(const dReal *inGenVelAB, dReal *genForceAB);
// computes restoring wrench
void compRestoringWrench(const dReal *inGenPosA, dReal *genForceAB);
// computes Coriolis added
void compCoriolisCor(const dReal *inGenVelAB, dReal *genForceAB);

//----------------------------------------------------------------------------//
// Model functions

void bmod_ComputeGenForces(const dReal *inGenPosA, const dReal *inGenVelAB, const dReal *inPropThrusts, dReal *outGenForceAB) {
  // Set GenForceAB to zero
  utils_SetZero(outGenForceAB, 6);

  compRestoringWrench(inGenPosA, outGenForceAB);

  b2b_compThrustWrench(inPropThrusts, outGenForceAB);

  compDampingWrench(inGenVelAB, outGenForceAB);

  // Coriolis correction (due to added mass (not in ODE -> not in Webots))
  compCoriolisCor(inGenVelAB, outGenForceAB);
};

void compDampingWrench(const dReal *inGenVelAB, dReal *genForceAB) {
  int i;

  for (i = 0; i < 6; i++) {
    // Linear damping
    genForceAB[i] += -b2b_LinearDamp[i] * inGenVelAB[i];

    // Quadratic damping
    genForceAB[i] += -b2b_QuadraticDamp[i] * dFabs(inGenVelAB[i]) * inGenVelAB[i];
  }
}

void compRestoringWrench(const dReal *inGenPosA, dReal *genForceAB) {
  // Restoring moment
  genForceAB[3] += -b2b_m * g * b2b_rz * dCos(inGenPosA[4]) * dSin(inGenPosA[3]);
  genForceAB[4] += -b2b_m * g * b2b_rz * dSin(inGenPosA[4]);
  genForceAB[5] += 0;
};

void compCoriolisCor(const dReal *inGenVelAB, dReal *genForceAB) {
  int i;

  // Some of these products are not "cross" product, and
  // it seemed better to me to let it so, to see what's really done. It would
  // be much better if it was written already in ODE, because here, we have to
  // compute all OUR Coriolis gen. forces, and after, deduct WEBOTS Coriolis
  // gen. forces! And we have to be carefull with the mass in each direction,
  // because, for example, works with mass b2b_m in x direction, and we are
  // working with mass b2b_m + addedMass[0] in x direction!!!

  // Coriolis (rigid body classical Coriolis gen. forces)
  genForceAB[0] += -b2b_m * inGenVelAB[2] * inGenVelAB[4] + b2b_m * inGenVelAB[1] * inGenVelAB[5];
  genForceAB[1] += +b2b_m * inGenVelAB[2] * inGenVelAB[3] - b2b_m * inGenVelAB[0] * inGenVelAB[5];
  genForceAB[2] += -b2b_m * inGenVelAB[1] * inGenVelAB[3] + b2b_m * inGenVelAB[0] * inGenVelAB[4];
  // Coriolis added mass (added mass Coriolis gen. forces)
  genForceAB[0] += -b2b_addedMass[2] * inGenVelAB[2] * inGenVelAB[4] + b2b_addedMass[1] * inGenVelAB[1] * inGenVelAB[5];
  genForceAB[1] += +b2b_addedMass[2] * inGenVelAB[2] * inGenVelAB[3] - b2b_addedMass[0] * inGenVelAB[0] * inGenVelAB[5];
  genForceAB[2] += -b2b_addedMass[1] * inGenVelAB[1] * inGenVelAB[3] + b2b_addedMass[0] * inGenVelAB[0] * inGenVelAB[4];
  genForceAB[3] += -b2b_addedMass[2] * inGenVelAB[2] * inGenVelAB[1] + b2b_addedMass[1] * inGenVelAB[1] * inGenVelAB[2];
  genForceAB[4] += +b2b_addedMass[2] * inGenVelAB[2] * inGenVelAB[0] - b2b_addedMass[0] * inGenVelAB[0] * inGenVelAB[2];
  genForceAB[5] += -b2b_addedMass[1] * inGenVelAB[1] * inGenVelAB[0] + b2b_addedMass[0] * inGenVelAB[0] * inGenVelAB[1];
  // Added mass correction from Webots (kill the term b2b_m, because we are
  // working with the terms b2b_m + addedMass[i] in i direction (i=0,1,2))
  for (i = 0; i < 3; i++) {
    genForceAB[i] = genForceAB[i] * b2b_m / (b2b_m + b2b_addedMass[i]);
  }
  // Deduct Coriolis that will be added by Webots ("kill" Coriolis normal
  // rigid body gen. force from Webots)
  genForceAB[0] -= -b2b_m * inGenVelAB[2] * inGenVelAB[4] + b2b_m * inGenVelAB[1] * inGenVelAB[5];
  genForceAB[1] -= +b2b_m * inGenVelAB[2] * inGenVelAB[3] - b2b_m * inGenVelAB[0] * inGenVelAB[5];
  genForceAB[2] -= -b2b_m * inGenVelAB[1] * inGenVelAB[3] + b2b_m * inGenVelAB[0] * inGenVelAB[4];
};
