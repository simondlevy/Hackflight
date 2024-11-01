#include <stdio.h>
#include <sys/time.h>

#include <hackflight.hpp>
#include <pids/altitude.hpp>
#include <timer.hpp>
#include <sim/vehicles/tinyquad.hpp>

static const float INITIAL_ALTITUDE_TARGET = 0.2;

static const float THRUST_BASE = 55.385;

static const float DYNAMICS_DT = 1e-5;

static const float DYNAMICS_FREQ = 1e5;
static const float PID_FREQ = 1e3;
static const float REPORT_FREQ = 1e2;

static const float MOTOR_MAX = 60;

static const float TIME_MAX = 5;

static float min(const float val, const float maxval)
{
    return val > maxval ? maxval : val;
}

static uint32_t usec()
{
    struct timeval tv = {};
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1'000'000 + tv.tv_usec;
}

int main(int argc, char ** argv)
{
    Dynamics dynamics = Dynamics(tinyquad_params, DYNAMICS_DT);

    hf::AltitudePid altitudePid = {};

    hf::state_t state  = {};

    hf::demands_t demands = {};

    hf::Timer dynamics_timer;
    hf::Timer pid_timer;
    hf::Timer report_timer;

    for (long k=0; ; k++) {

        const auto usec_curr = usec();

        const auto time = k * DYNAMICS_DT;

        float motor = 0;

        if (time >= TIME_MAX) {
            break;
        }

        if (true) {

            // Reset thrust demand to altitude target
            demands.thrust = INITIAL_ALTITUDE_TARGET;

            // Altitude PID controller converts target to thrust demand
            altitudePid.run(DYNAMICS_DT, state, demands);

            motor = min(demands.thrust + THRUST_BASE, MOTOR_MAX);

        }

        if (true) {
            dynamics.setMotors(motor, motor, motor, motor);
            state.z = dynamics.x[Dynamics::STATE_Z];
            state.dz = dynamics.x[Dynamics::STATE_Z_DOT];
        }

        if (true) {
            printf("%3.3f,%3.3f,%3.3f,%3.3f\n",
                    time, motor, state.z, state.dz);
        }
    }

    return 0;
}
