#include <stdio.h>

#include "Dynamics.hpp"
#include "haskell.h"

float stream_m1;
float stream_m2;
float stream_m3;
float stream_m4;
float stream_time;
float stream_agl;

Dynamics::vehicle_params_t vparams = {

    // Estimated
    2.e-06, // d drag cofficient [T=d*w^2]

    // https://www.dji.com/phantom-4/info
    1.380,  // m mass [kg]

    // Estimated
    2,      // Ix [kg*m^2] 
    2,      // Iy [kg*m^2] 
    3,      // Iz [kg*m^2] 
    3.8e-03, // Jr prop inertial [kg*m^2] 
    15000,   // maxrpm
};

Dynamics::fixed_pitch_params_t fpparams = {
    5.e-06, // b thrust coefficient [F=b*w^2]
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

    stream_m1 = 0.6;
    stream_m2 = 0.6;
    stream_m3 = 0.4;
    stream_m4 = 0.4;

    float motors[4] = {stream_m1, stream_m2, stream_m3, stream_m4};

    Dynamics::state_t state = {};

    stream_agl = 0;

    for (int k=0; k<10; ++k) {

        stream_time = k / 1000.;

        dynamics.update(motors, state, stream_agl, stream_time);

        stream_agl = -state.z;

        step();

        printf("t=%f  %+6.6f | %+6.6f\n", stream_time, state.psi, _psi);
    }

    return 0;
}
