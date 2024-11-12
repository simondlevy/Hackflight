#include <stdio.h>
#include <stdio.h>
#include <stdint.h>

#include "dynamics/fixedpitch/QuadXBF.hpp"

// Time constant
static const double DT = 2.0e-5;

// Vehicle constants

static Dynamics::vehicle_params_t vparams = {

    2.e-06,  // drag coefficient [T=d*w^2]
    0.05,    // m mass [kg]
    2,       // Ix [kg*m^2] 
    2,       // Iy [kg*m^2] 
    3,       // Iz [kg*m^2] 
    3.8e-03  // Jr prop inertial [kg*m^2] 
};

static FixedPitchDynamics::fixed_pitch_params_t fparams = {

    // Estimated
    3.275e-5, // b force constatnt [F=b*w^2]
    0.03    // l arm length [m]
};

int main(int argc, char ** argv)
{
    // Create quadcopter dynamics model
    QuadXBFDynamics dynamics =
        QuadXBFDynamics(vparams, fparams, false); // no auto-land

    // Set up initial conditions
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    // Loop forever, communicating with server
    for (uint64_t k=0; ; ++k) {

        const double time = k * DT;

        const float MOTOR = 55.385; // rad/sec
        float motorvals[4] = {MOTOR, MOTOR, MOTOR, MOTOR};

        printf("t=%05f   m=%f %f %f %f  z=%+3.3f\n", 
                time,
                motorvals[0],
                motorvals[1],
                motorvals[2],
                motorvals[3],
                dynamics.state.z);

        // Update dynamics with motor values
        dynamics.update(motorvals, DT);

        // Set AGL to arbitrary positive value to avoid kinematic trick
        dynamics.setAgl(1);
    }

    return 0;
}
