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
#include <sim.hpp>

static const float THRUST_TAKEOFF = 56;
static const float THRUST_BASE = 55.385;

static const float TAKEOFF_TIME = 3;

static const float YAW_PREDIVISOR = 160; // deg/sec

static const float YAW_DIVISOR  = 26;
static const float YAW_OFFSET = 0.955;

static const float CLIMBRATE_DIVISOR  = 3;
static const float CLIMBRATE_OFFSET = 8.165;

static double runClimbrateSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};

    snn->step(observations, counts);

    return counts[0] / CLIMBRATE_DIVISOR - CLIMBRATE_OFFSET;
}

static double runYawSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};

    snn->step(observations, counts);

    return counts[0] / YAW_DIVISOR - YAW_OFFSET;
}

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init(false);

    SNN * climbRateSnn = NULL;

    SNN * yawRateSnn = NULL;

    // Load up the network specified in the command line

    if (argc < 2) {
        fprintf(stderr, "Usage: %s RISP_NETWORK VIZ_PORT]\n", argv[0]);
        exit(1);
    }

    try {

        climbRateSnn = new SNN(argv[1], "risp");
        yawRateSnn = new SNN(argv[1], "risp");

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    const auto viz_port = argc > 2 ? atoi(argv[2]) : 0;

    if (viz_port) {
        climbRateSnn->serve_visualizer(viz_port);
    }

    while (true) {

        if (!sim.step()) {
            break;
        }

        const auto state = sim.getState();

        auto demands = sim.getDemandsFromKeyboard();

        const auto thrustFromSnn = runClimbrateSnn(
                climbRateSnn, demands.thrust, state.dz);

        demands.yaw = runYawSnn(
                yawRateSnn, demands.yaw, state.dpsi/YAW_PREDIVISOR);

        auto rollDemand = 6 * (10 * (demands.roll - state.dy) - state.phi);

        rollDemand = 0.0125 * (rollDemand - state.dphi);

        float pitchDemand  = 10 * (demands.pitch - state.dx);
        pitchDemand = 6 * (pitchDemand - state.theta);
        pitchDemand = 0.0125 * (pitchDemand - state.dtheta);

        // Ignore thrust demand until airborne, based on time from launch
        demands.thrust =
            sim.time() > TAKEOFF_TIME ? 
            thrustFromSnn + THRUST_BASE:
            sim.requestedTakeoff() ? 
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
