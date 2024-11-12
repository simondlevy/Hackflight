#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>

#include "dynamics.hpp"

// Vehicle constants

static const hf::Dynamics::vehicle_params_t vparams = {

    3.264065e-5, // b thrust coefficient [F=b*w^2]
    0.03,        // l arm length [m]

    2.e-06,      // drag coefficient [T=d*w^2]
    0.05,        // m mass [kg]
    2,           // Ix [kg*m^2] 
    2,           // Iy [kg*m^2] 
    3,           // Iz [kg*m^2] 
    3.8e-03      // Jr prop inertial [kg*m^2] 
};

int main(int argc, char ** argv)
{
    // Create quadcopter dynamics model
    auto dynamics = hf::Dynamics(vparams); 

    // Set up initial conditions
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    double time_prev = 0;

    // Loop forever
    while (true) {

        const float MOTOR = 55.385; // rad/sec
        float motorvals[4] = {MOTOR, MOTOR, MOTOR, MOTOR};

        // Update dynamics with motor values
        dynamics.update(motorvals);

        const auto time_curr = dynamics.getTime();

        if (time_curr - time_prev >= 1) {

            time_prev = time_curr;

            const auto state = dynamics.getState();

            if (time_curr > 0) {

                printf("t=%05f   m=%f %f %f %f  z=%+3.3f\n", 
                        time_curr,
                        motorvals[0],
                        motorvals[1],
                        motorvals[2],
                        motorvals[3],
                        state.z);
            }
        }
    }

    return 0;
}
