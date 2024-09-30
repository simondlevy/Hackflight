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

static const float TAKEOFF_TIME = 3;

static const float CLIMBRATE_SCALE  = 0.3333;
static const float CLIMBRATE_OFFSET = 47.22;

static const float YAW_KP = 0.48;           
static const float YAW_PRESCALE = 160; // deg/sec

static const float YAW_SNN_SCALE  = 12.5;
static const float YAW_SNN_OFFSET = 1.99;

static const float STICK_EPSILON = 0.001;

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

    return counts[0] / scale - offset;
}

static float runClimbRateSnn(
        SNN * snn,
        const float setpoint,
        const float actual,
        const float scale,
        const float offset,
        int & count)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts = {};
    snn->step(observations, counts);

    count = counts[0];

    return scale * count + offset;
}

static double cap(const float val, const float max)
{
    return val < -max ? -max : val > max ? max : val;
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

        if (!sim.step()) {
            break;
        }

        const auto time = sim.hitTakeoffButton() ? sim.time() : 0;

        const auto throttleCapped = cap(sim.throttle(), 1-STICK_EPSILON);

        int thrust_counts = 0;

        const auto thrustFromSnn = runClimbRateSnn(
                    climbRateSnn,
                    throttleCapped, 
                    sim.dz(),
                    CLIMBRATE_SCALE,
                    CLIMBRATE_OFFSET, 
                    thrust_counts);

        const auto airborne = time > TAKEOFF_TIME;

        if (airborne) {
            printf("%f,%f,%f,%f,%d\n",
                    time, throttleCapped, sim.dz(), thrustFromSnn, thrust_counts);
        }

        const auto yawDemand = YAW_KP * runDifferenceSnn(
                yawRateSnn,
                sim.yaw(),
                sim.dpsi()/YAW_PRESCALE,
                YAW_SNN_SCALE,
                YAW_SNN_OFFSET);

        auto rollDemand = 6 * (10 * (sim.roll() - sim.dy()) - sim.phi());

        rollDemand = 0.0125 * (rollDemand - sim.dphi());

        float pitchDemand  = 10 * (sim.pitch() - sim.dx());
        pitchDemand = 6 * (pitchDemand - sim.theta());
        pitchDemand = 0.0125 * (pitchDemand - sim.dtheta());

        // Ignore thrust demand until airborne, based on time from launch
        const auto thrustDemand =
            time > TAKEOFF_TIME ? 
            thrustFromSnn :
            sim.hitTakeoffButton() ? 
            THRUST_TAKEOFF :
            0;

        rollDemand = 0;
        pitchDemand = 0;

        // Run the mixer to convert the demands into motor spins
        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runBetaFlightQuadX(
                thrustDemand,
                rollDemand,
                pitchDemand,
                yawDemand,
                m1, m2, m3, m4);

        // Spin the motors
        sim.setMotors(m1, m2, m3, m4);

        if (viz_port) {
            climbRateSnn->send_counts_to_visualizer();
        }
    }

    wb_robot_cleanup();

    return 0;
}
