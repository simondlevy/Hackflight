#include <stdio.h>

#include <hackflight.hpp>
#include <pids/altitude.hpp>
#include <sim/vehicles/tinyquad.hpp>

static const float INITIAL_ALTITUDE_TARGET = 0.2;

static const float THRUST_BASE = 55.385;

static const uint32_t DYNAMICS_FREQ = 100'000;

static const float MOTOR_MAX = 60;

static const float TIME_MAX = 5;

static float min(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

int main(int argc, char ** argv)
{
    const float dynamics_dt = 1. / DYNAMICS_FREQ;

    Dynamics dynamics = Dynamics(tinyquad_params, dynamics_dt);

    hf::AltitudePid altitudePid = {};

    hf::state_t state  = {};

    hf::demands_t demands = {};

    for (long k=0; ; k++) {

        const auto time = k / (float)DYNAMICS_FREQ;

        float motor = 0;

        if (time >= TIME_MAX) {
            break;
        }

        if (true) {

            // Reset thrust demand to altitude target
            demands.thrust = INITIAL_ALTITUDE_TARGET;

            // Altitude PID controller converts target to thrust demand
            altitudePid.run(dynamics_dt, state, demands);

            motor = min(demands.thrust + THRUST_BASE, MOTOR_MAX);

        }

        dynamics.setMotors(motor, motor, motor, motor);
        state.z = dynamics.x[Dynamics::STATE_Z];
        state.dz = dynamics.x[Dynamics::STATE_Z_DOT];

        if (k % 100 == 0) {
            printf("%3.3f,%3.3f,%3.3f,%3.3f\n",
                    time, motor, state.z, state.dz);
        }
    }

    return 0;
}
