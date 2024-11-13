#include "quadrotor2b.h"

static const dReal SinAlphaTimesdAlpha = REAL(0.144);
static const dReal SinBetaTimesdBeta = REAL(0.073);
static const dReal CosBetaTimesdBeta = REAL(0.484);
static const dReal SinGammaTimesdGamma = REAL(0.045);

//------------------------------------------------------------------------------
// Engines: command [-1.0...1.0] to thrust [N] converters

void b2b_commandsToThrust(double inFront, double inYaw, double inVert, dReal *propThrusts) {
  // our thrusters do not have same power in both direction due to propeller shape.
  propThrusts[0] = inFront * (inFront < 0 ? REAL(0.0450) : REAL(0.0400));
  propThrusts[1] = inYaw * (inYaw < 0 ? REAL(0.0250) : REAL(0.0300));
  propThrusts[2] = inVert * (inVert < 0 ? REAL(0.0450) : REAL(0.0250));
}

void b2b_compThrustWrench(const dReal *propThrusts, dReal *genForceAB) {
  genForceAB[0] += +propThrusts[0];
  genForceAB[1] += -propThrusts[1];
  genForceAB[2] += +propThrusts[2];
  genForceAB[3] += +SinBetaTimesdBeta * propThrusts[1];
  genForceAB[4] += +SinAlphaTimesdAlpha * propThrusts[0] - SinGammaTimesdGamma * propThrusts[2];
  genForceAB[5] += +CosBetaTimesdBeta * propThrusts[1];
}
