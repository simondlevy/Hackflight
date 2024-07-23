#include <tasks/core.hpp>
#include <webots.hpp>

static const float PITCH_ROLL_ANGLE_KP = 6e0;

static const float PITCH_ROLL_RATE_KP = 1.25e-2;
static const float PITCH_ROLL_RATE_KD = 1.25e-4;

static const float YAW_RATE_KP = 1.20e-2;

// Motor thrust constants for climb-rate PID controller
static const float TBASE = 56;
static const float TSCALE = 0.25;
static const float TMIN = 0;

// Arbitrary time constexprant
static constexpr float DT = .01;

int main(int argc, char ** argv)
{
    CoreTask coreTask = {};

    coreTask.init(
            PITCH_ROLL_ANGLE_KP, 
            PITCH_ROLL_RATE_KP, 
            PITCH_ROLL_RATE_KD,
            YAW_RATE_KP, 
            TBASE, 
            TSCALE, 
            TMIN,
            DT);

    Simulator sim = {};

    sim.init();

    state_t state = {};

    demands_t demands = {};

    while (sim.step(demands, state)) {

        quad_motors_t motors = {};

        coreTask.run(state, demands, motors);

        sim.setMotors(motors.m1, motors.m2, motors.m3, motors.m4);
    }

    sim.close();

    return 0;
}
