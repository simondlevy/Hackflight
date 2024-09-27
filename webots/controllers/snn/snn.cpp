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

//static const int CLIMBRATE_VIZ_PORT = 8100;
//static const int YAWRATE_VIZ_PORT = 8200;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.5;

static const float TAKEOFF_TIME = 3;

static const float CLIMBRATE_KP = 4;
static const float CLIMBRATE_PRESCALE = 1.0;

static const float YAW_KP = 0.003;           
static const float YAW_PRESCALE = 160; // deg/sec

static const float YAW_OFFSET = 0.01;

static float runDifferenceSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = { setpoint, actual };

    vector <int> counts;
    vector <double> actions;
    snn->step(observations, counts, actions);

    // NEGATE because our SNN used flip=true
    return -actions[0];
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
        fprintf(stderr, "Usage: %s RISP_NETWORK\n", argv[0]);
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

    //climbRateSnn->serve_visualizer(CLIMBRATE_VIZ_PORT);
    //yawRateSnn->serve_visualizer(YAWRATE_VIZ_PORT);

    while (true) {

        if (!sim.step()) {
            break;
        }

        // Get thrust demand from SNN
        const auto thrustFromSnn =
            CLIMBRATE_KP * runDifferenceSnn(
                    climbRateSnn,
                    CLIMBRATE_PRESCALE*sim.throttle(),
                    CLIMBRATE_PRESCALE*sim.dz());

        const auto time = sim.hitTakeoffButton() ? sim.time() : 0;

        const auto yawDemand = YAW_KP * YAW_PRESCALE * runDifferenceSnn(
                yawRateSnn,
                sim.yaw(),
                sim.dpsi()/YAW_PRESCALE) + YAW_OFFSET;

        auto rollDemand = 6 * (10 * (sim.roll() - sim.dy()) - sim.phi());

        rollDemand = 0.0125 * (rollDemand - sim.dphi());

        float pitchDemand  = 10 * (sim.pitch() - sim.dx());
        pitchDemand = 6 * (pitchDemand - sim.theta());
        pitchDemand = 0.0125 * (pitchDemand - sim.dtheta());

        // Ignore thrust demand until airborne, based on time from launch
        const auto thrustDemand =
            time > TAKEOFF_TIME ? 
            THRUST_BASE + thrustFromSnn :
            sim.hitTakeoffButton() ? 
            THRUST_TAKEOFF :
            0;

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

        //climbRateSnn->send_counts_to_visualizer();
        //yawRateSnn->send_counts_to_visualizer();
    }

    wb_robot_cleanup();

    return 0;
}
