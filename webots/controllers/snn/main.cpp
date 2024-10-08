/*
   C++ flight simulator takeoff example for Hackflight

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

// TeNNLab framework
#include <levy_snn_util.hpp>

// Hackflight
#include <mixers.hpp>
#include <oldsim.hpp>

static const float THRUST_TAKEOFF = 56;

static const float TAKEOFF_TIME = 3;

static const float CLIMBRATE_SNN_SCALE  = 0.3333;
static const float CLIMBRATE_SNN_OFFSET = 47.22;

static const float YAW_PRESCALE = 160; // deg/sec

static const float YAW_SNN_SCALE  = 0.0384;
static const float YAW_SNN_OFFSET = -0.955;

static double runDifferenceSnn(
        SNN * snn,
        const float setpoint,
        const float actual,
        const float scale,
        const float offset)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};

    snn->step(observations, counts);

    return scale * counts[0] + offset;
}

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

    SNN * climbRateSnn = NULL;

    SNN * yawRateSnn = NULL;
    //SNN * positionYSnn = NULL;
    //SNN * rollAngleSnn = NULL;
    //SNN * rollRateSnn = NULL;

    // Load up the network specified in the command line

    if (argc < 2) {
        fprintf(stderr, "Usage: %s RISP_NETWORK VIZ_PORT]\n", argv[0]);
        exit(1);
    }

    try {

        climbRateSnn = new SNN(argv[1], "risp");
        yawRateSnn = new SNN(argv[1], "risp");
        //positionYSnn = new SNN(argv[1], "risp");
        //rollAngleSnn = new SNN(argv[1], "risp");
        //rollRateSnn = new SNN(argv[1], "risp");

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    const auto viz_port = argc > 2 ? atoi(argv[2]) : 0;

    if (viz_port) {
        climbRateSnn->serve_visualizer(viz_port);
    }

    while (true) {

        hf::demands_t demands = {};
        hf::state_t state = {};
        bool requestedTakeoff = false;

        if (!sim.step(state, demands, requestedTakeoff)) {
            break;
        }

        const auto time = requestedTakeoff ? sim.time() : 0;

        const auto thrustFromSnn = runDifferenceSnn(
                    climbRateSnn,
                    demands.thrust, 
                    state.dz,
                    CLIMBRATE_SNN_SCALE,
                    CLIMBRATE_SNN_OFFSET) ;

        const auto airborne = time > TAKEOFF_TIME;

        if (airborne) {
            printf("%f,%f,%f,%f\n",
                    time, demands.thrust, state.dz, thrustFromSnn);
        }

        demands.yaw = runDifferenceSnn(
                yawRateSnn,
                demands.yaw,
                state.dpsi/YAW_PRESCALE,
                YAW_SNN_SCALE,
                YAW_SNN_OFFSET);

        auto rollDemand = 6 * (10 * (demands.roll - state.dy) - state.phi);

        rollDemand = 0.0125 * (rollDemand - state.dphi);

        float pitchDemand  = 10 * (demands.pitch - state.dx);
        pitchDemand = 6 * (pitchDemand - state.theta);
        pitchDemand = 0.0125 * (pitchDemand - state.dtheta);

        // Ignore thrust demand until airborne, based on time from launch
        demands.thrust =
            time > TAKEOFF_TIME ? 
            thrustFromSnn :
            requestedTakeoff ? 
            THRUST_TAKEOFF :
            0;

        demands.pitch = 0;
        demands.roll = 0;

        hf::quad_motors_t motors= {};

        hf::Mixer::runBetaFlightQuadX(demands, motors);

        sim.setMotors(motors);

        if (viz_port) {
            climbRateSnn->send_counts_to_visualizer();
        }
    }

    wb_robot_cleanup();

    return 0;
}
