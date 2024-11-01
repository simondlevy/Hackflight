#include <sys/time.h>

#include <hackflight.hpp>
#include <pids/altitude.hpp>
#include <sim/vehicles/tinyquad.hpp>

static constexpr float DYNAMICS_DT = 1e-5;

static const float THRUST_BASE = 55.385;

static const float THROTTLE_DEADBAND = 0.2;

// For springy-throttle gamepads / keyboard
static const float INITIAL_ALTITUDE_TARGET = 0.2;
static const float CLIMB_RATE_SCALE = 0.01;

static double timesec()
{
    struct timeval tv = {};
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1e6;

}

int main(int argc, char ** argv)
{

    double time_prev = 0;
    bool ready = false;

    Dynamics _dynamics = Dynamics(tinyquad_params, DYNAMICS_DT);

    while (true) {

        const auto time_curr = timesec();

        if (time_curr - time_prev > DYNAMICS_DT) {

            if (ready) {

                time_prev = time_curr;
            }

            ready  = true;
        }
    }

    return 0;
}
