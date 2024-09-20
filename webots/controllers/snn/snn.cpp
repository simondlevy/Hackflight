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

static const float K2 = 6;

static const float CLIMBRATE_KP = 4;
static const float CLIMBRATE_PRESCALE = 1.0;

static const float YAW_KP = 0.003;           
static const float YAW_PRESCALE = 160; // deg/sec

static const float DT = 0.01;

static const float THROTTLE_DOWN = 0.06;

static const float K1 = 0.0125;

static const float K3 = 10;

static double runYawRateSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = {
        setpoint,
        actual / YAW_PRESCALE
    };

    vector <double> actions;
    snn->step(observations, actions);

    // NEGATE because our SNN used flip=true
    return -actions[0] * YAW_KP * YAW_PRESCALE;
}

static float runDifferenceSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = { setpoint, actual };

    vector <double> actions;
    snn->step(observations, actions);

    // NEGATE because our SNN used flip=true
    return -actions[0];
}


static float runCascade(
        const float stick, const float vel, const float angle, const float dangle)
{
    return ((stick - vel) - angle) - dangle;
}

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

    SNN * climbrate_snn = NULL;
    SNN * yawrate_snn = NULL;
    SNN * rollangle_snn = NULL;
    SNN * rollrate_snn = NULL;

    // Load up the network specified in the command line

    if (argc < 2) {
        fprintf(stderr, "Usage: %s RISP_NETWORK\n", argv[0]);
        exit(1);
    }

    try {

        climbrate_snn = new SNN(argv[1], "risp");
        yawrate_snn = new SNN(argv[1], "risp");
        rollangle_snn = new SNN(argv[1], "risp");
        rollrate_snn = new SNN(argv[1], "risp");

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    //climbrate_snn->serve_visualizer(CLIMBRATE_VIZ_PORT);
    //yawrate_snn->serve_visualizer(YAWRATE_VIZ_PORT);

    while (true) {

        if (!sim.step()) {
            break;
        }

        // Get thrust demand from SNN
        const auto thrustFromSnn =
            CLIMBRATE_KP * runDifferenceSnn(
                    climbrate_snn,
                    CLIMBRATE_PRESCALE*sim.throttle(),
                    CLIMBRATE_PRESCALE*sim.dz());

        // Get yaw demand from SNN
        const auto yawDemand = runYawRateSnn(yawrate_snn, sim.yaw(), sim.dpsi());

        //const auto rollDemand = K1*K2*K3 *
        //    runCascade(sim.roll(), sim.dy(), sim.phi()/K3, sim.dphi()/(K2*K3));

        auto rollDemand = sim.roll() - sim.dy();

        (void)rollangle_snn;

        rollDemand = rollDemand - sim.phi()/K2;

        rollDemand = runDifferenceSnn(rollrate_snn, rollDemand, sim.dphi() / (K2 * K3));

        rollDemand = K1*K2*K3 * rollDemand;

        const auto pitchDemand = K1*K2*K3 *
            runCascade(sim.pitch(), sim.dx(), sim.theta()/K3, sim.dtheta()/(K2*K3));

        // Ignore thrust demand until airborne, based on time from launch
        const auto time = sim.hitTakeoffButton() ? sim.time() : 0;
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

        //climbrate_snn->send_counts_to_visualizer();
        //yawrate_snn->send_counts_to_visualizer();
    }

    wb_robot_cleanup();

    return 0;
}
