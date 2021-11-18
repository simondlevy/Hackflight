#include <stdio.h>
#include <Dynamics.hpp>
#include "haskell.h"

float stream_time;
float stream_agl;

Dynamics::vehicle_params_t vparams = {

    // Estimated
    2.E-06, // d drag cofficient [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // m mass [kg]

    // Estimated
    2,      // Ix [kg*m^2] 
    2,      // Iy [kg*m^2] 
    3,      // Iz [kg*m^2] 
    3.8E-03, // Jr prop inertial [kg*m^2] 
    15000,   // maxrpm
};

Dynamics::fixed_pitch_params_t fpparams = {
    5.E-06, // b thrust coefficient [F=b*w^2]
    0.350   // l arm length [m]
};

static float _x;
static float _y;
static float _z;
static float _phi;
static float _theta;
static float _psi;

void stream_debug(float x, float y, float z, float phi, float theta, float psi)
{
    _x     = x;
    _y     = y;
    _z     = z;
    _phi   = phi;
    _theta = theta;
    _psi   = psi;
}

int main(int argc, char ** argv)
{
    Dynamics dynamics = Dynamics(vparams, fpparams);

    float M = 1.0;

    float motors[4] = {M, M, M, M};

    Dynamics::state_t state = {};

    stream_agl = 0;

    for (int k=0; k<20; ++k) {

        stream_time = k / 1000.;

        dynamics.update(motors, state, stream_agl, stream_time);

        stream_agl = -state.z;

        step();

        printf("t=%f  x=%+6.6f | %+6.6f\n", stream_time, state.z, _z);
    }

    return 0;
}
