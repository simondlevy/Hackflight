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

static const float CLIMBRATE_KP = 4;
static const float CLIMBRATE_PRESCALE = 1.0;

static const float CLIMBRATE_SNN_SCALE_TRAINED  = 12.0;
static const float CLIMBRATE_SNN_OFFSET_TRAINED = 2.07;

static const float CLIMBRATE_SNN_SCALE_BYHAND  = 12.0;
static const float CLIMBRATE_SNN_OFFSET_BYHAND = 1.95;

static const float THRUST_BASE = 55.5;

static const float YAW_KP = 0.003;           
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

    const double action = count / scale - offset;

    return action;
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

    SNN * climbRateSnn_trained = NULL;
    SNN * climbRateSnn_byhand = NULL;

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

        climbRateSnn_trained = new SNN(argv[1], "risp");
        yawRateSnn = new SNN(argv[1], "risp");
        //positionYSnn = new SNN(argv[1], "risp");
        //rollAngleSnn = new SNN(argv[1], "risp");
        //rollRateSnn = new SNN(argv[1], "risp");

        if (argc > 2) {
            climbRateSnn_byhand = new SNN(argv[2], "risp");
        }

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    const auto viz_port = argc > 2 ? atoi(argv[2]) : 0;

    if (viz_port) {
        climbRateSnn_trained->serve_visualizer(viz_port);
    }

    while (true) {

        if (!sim.step()) {
            break;
        }

        const auto time = sim.hitTakeoffButton() ? sim.time() : 0;

        const auto airborne = time > TAKEOFF_TIME;

        const auto throttleScaled = cap(CLIMBRATE_PRESCALE*sim.throttle(), 1-STICK_EPSILON);

        const auto climbRateScaled = CLIMBRATE_PRESCALE*sim.dz();

        int trained_counts = 0;

        const auto thrust_trained_scaled = runClimbRateSnn(
                    climbRateSnn_trained,
                    throttleScaled, 
                    climbRateScaled,
                    CLIMBRATE_SNN_SCALE_TRAINED,
                    CLIMBRATE_SNN_OFFSET_TRAINED, 
                    trained_counts);

        if (airborne) {
            printf("%f,%f,%f,%f,%d",
                    time, throttleScaled, climbRateScaled, thrust_trained_scaled, trained_counts);
        }

        int byhand_counts = 0;

        if (argc > 2) {
            const auto thrust_byhand_scaled = runClimbRateSnn(
                    climbRateSnn_byhand,
                    throttleScaled, 
                    climbRateScaled,
                    CLIMBRATE_SNN_SCALE_BYHAND,
                    CLIMBRATE_SNN_OFFSET_BYHAND,
                    byhand_counts);

            if (airborne) {
                printf(",%f,%d", thrust_byhand_scaled, byhand_counts);
            }
        }

        if (airborne) {
            printf("\n");
            fflush(stdout);
        }

        const auto thrustFromSnn = thrust_trained_scaled * CLIMBRATE_KP;

        auto yawDemand = YAW_KP * YAW_PRESCALE * runDifferenceSnn(
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
            THRUST_BASE + thrustFromSnn :
            sim.hitTakeoffButton() ? 
            THRUST_TAKEOFF :
            0;

        rollDemand = 0;
        pitchDemand = 0;
        yawDemand = 0;

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
            climbRateSnn_trained->send_counts_to_visualizer();
        }
    }

    wb_robot_cleanup();

    return 0;
}
