/*
   Spiking Neural Net controller for Hackflight

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

static const float YAW_PRE_DIVISOR = 160; // deg/sec

static const float PITCH_ROLL_PRE_DIVISOR = 10; // deg

static const float YAW_DIVISOR  = 26;
static const float YAW_OFFSET = 0.955;

static const float CLIMBRATE_DIVISOR  = 3;
static const float CLIMBRATE_OFFSET = 8.165;

static const float CASCADE_DIVISOR  = 15;
static const float CASCADE_OFFSET = 0.936;
static const float CASCADE_POST_SCALE = 120;

static const float PITCH_ROLL_POST_SCALE = 50;

static const char * NETWORK = "networks/difference_risp_train.txt";

static const char * DYDT_NETWORK = "networks/dydt_risp.txt";

static const char * NETWORK3 = "networks/difference3_risp.txt";

static double runSnn(
        SNN * snn,
        const float setpoint,
        const float actual,
        const float divisor,
        const float offset,
        const bool debug=false)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};

    snn->step(observations, counts);

    const double action = counts[0] / divisor - offset;

    if (debug) {
        printf("%f,%f\n", setpoint - actual, action);
    }

    return action;
}

static float runCascadeSnn(
        SNN * snn, const float inp1, const float inp2, const float inp3)
{
    vector<double> observations = { inp1, inp2, inp3 };

    vector <int> counts = {};

    snn->step(observations, counts);

    return counts[0] / CASCADE_DIVISOR - CASCADE_OFFSET;
}

int main(int argc, char ** argv)
{
    hf::Simulator sim = {};

    sim.init(false);

    SNN * climbRateSnn = NULL;

    SNN * yawRateSnn = NULL;

    SNN * dydtSnn = NULL;

    SNN * cascadeSnn = NULL;

    SNN * vizSnn = NULL;

    // Load up the network specified in the command line

    try {

        climbRateSnn = new SNN(NETWORK, "risp");
        yawRateSnn = new SNN(NETWORK, "risp");
        dydtSnn = new SNN(DYDT_NETWORK, "risp");
        cascadeSnn = new SNN(NETWORK3, "risp");

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    const auto viz_port = argc > 1 ? atoi(argv[1]) : 0;

    if (viz_port) {
        vizSnn = yawRateSnn;
        vizSnn->serve_visualizer(viz_port);
    }

    while (true) {

        if (!sim.step()) {
            break;
        }

        const auto state = sim.getState();

        auto demands = sim.getDemandsFromKeyboard();

        const auto thrustFromSnn = runSnn(
                climbRateSnn, demands.thrust, state.dz,
                CLIMBRATE_DIVISOR, CLIMBRATE_OFFSET);

        demands.yaw = runSnn(
                yawRateSnn, demands.yaw, state.dpsi/YAW_PRE_DIVISOR,
                YAW_DIVISOR, YAW_OFFSET);

        const auto phi = state.phi / PITCH_ROLL_PRE_DIVISOR;

        /*const auto rollAngleDemand = */runSnn(
                dydtSnn, demands.roll, state.dy,
                25, 0.9, true);

        const auto snn_diff = 
            runCascadeSnn(cascadeSnn, demands.roll, state.dy, phi);

        demands.roll = 60 * snn_diff;
        demands.roll = 0.0125 * (demands.roll - state.dphi);

        demands.pitch = 6 * (10 * (demands.pitch - state.dx) - state.theta);
        demands.pitch = 0.0125 * (demands.pitch - state.dtheta);

        // Ignore thrust demand until airborne, based on time from launch
        demands.thrust =
            sim.time() > TAKEOFF_TIME ? 
            thrustFromSnn + THRUST_BASE:
            sim.requestedTakeoff() ? 
            THRUST_TAKEOFF :
            0;

        hf::quad_motors_t motors= {};

        hf::Mixer::runBetaFlightQuadX(demands, motors);

        sim.setMotors(motors);

        if (viz_port) {
            vizSnn->send_counts_to_visualizer();
        }
    }

    wb_robot_cleanup();

    return 0;
}
