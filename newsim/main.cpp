#include <stdio.h>
#include <sys/time.h>

#include <hackflight.hpp>
#include <timer.hpp>
#include <pids/altitude.hpp>
#include <sim/vehicles/tinyquad.hpp>

static constexpr float DYNAMICS_FREQ = 100'000;

static constexpr float PID_FREQ = 1000;

static constexpr float REPORT_FREQ = 30;

static const float THRUST_BASE = 55.385;

static const float THROTTLE_DEADBAND = 0.2;

// For springy-throttle gamepads / keyboard
static const float INITIAL_ALTITUDE_TARGET = 0.2;
static const float CLIMB_RATE_SCALE = 0.01;

static uint32_t usec()
{
    struct timeval tv = {};
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1'000'000 + tv.tv_usec;
}

int main(int argc, char ** argv)
{

    uint32_t time_prev = 0;
    bool ready = false;

    Dynamics dynamics = Dynamics(tinyquad_params, 1./DYNAMICS_FREQ);

    hf::Timer dynamics_timer;
    hf::Timer pid_timer;
    hf::Timer report_timer;

    hf::AltitudePid altitudePid = {};

    hf::state_t state  = {};

    hf::demands_t demands = {INITIAL_ALTITUDE_TARGET, 0, 0, 0};

    float motor = 0;

    while (true) {

        const auto time_curr = usec();

        if (dynamics_timer.isReady(usec(), DYNAMICS_FREQ)) {

            dynamics.setMotors(motor, motor, motor, motor);

            state.z = dynamics.x[Dynamics::STATE_Z];
        }

        if (pid_timer.isReady(usec(), PID_FREQ)) {

            altitudePid.run(1./PID_FREQ, state, demands);

            motor = demands.thrust;
        }

        if (report_timer.isReady(usec(), REPORT_FREQ)) {

            printf("%3.3f\n", state.z);
        }
    }

    return 0;
}
