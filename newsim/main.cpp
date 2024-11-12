#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>

#include "dynamics.hpp"

static const double DYNAMICS_FREQ = 3.75e6;

// Vehicle constants

static const hf::Dynamics::vehicle_params_t vparams = {

    3.275e-5, // b thrust coefficient [F=b*w^2]
    0.03,   // l arm length [m]

    2.e-06,  // drag coefficient [T=d*w^2]
    0.05,    // m mass [kg]
    2,       // Ix [kg*m^2] 
    2,       // Iy [kg*m^2] 
    3,       // Iz [kg*m^2] 
    3.8e-03  // Jr prop inertial [kg*m^2] 
};

int main(int argc, char ** argv)
{
    // Create quadcopter dynamics model
    auto dynamics = hf::Dynamics(vparams); 

    // Set up initial conditions
    double rotation[3] = {0,0,0};
    dynamics.init(rotation);

    time_t sec_prev = 0;

    const auto dynamics_dt = 1 / DYNAMICS_FREQ;

    // Loop forever, communicating with server
    for (uint64_t k=0; ; ++k) {

        const double time = k * dynamics_dt;

        const float MOTOR = 55.385; // rad/sec
        float motorvals[4] = {MOTOR, MOTOR, MOTOR, MOTOR};

        struct timeval tv = {};
        gettimeofday(&tv, NULL);
        time_t sec_curr = tv.tv_sec;

        if (sec_curr - sec_prev > 0) {

            sec_prev = sec_curr;

            printf("t=%05f   m=%f %f %f %f  z=%+3.3f\n", 
                    time,
                    motorvals[0],
                    motorvals[1],
                    motorvals[2],
                    motorvals[3],
                    dynamics.state.z);
        }

        // Update dynamics with motor values
        dynamics.update(motorvals, dynamics_dt);
    }

    return 0;
}
