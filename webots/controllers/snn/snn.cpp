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
#include <pids/position.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>

//static const int CLIMBRATE_VIZ_PORT = 8100;
//static const int YAWRATE_VIZ_PORT = 8200;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.5;

static const float TAKEOFF_TIME = 3;

static const float CLIMBRATE_KP = 25;
static const float CLIMBRATE_PRESCALE = 0.5;

static const float YAW_KP = 0.003;           
static const float YAW_PRESCALE = 160; // deg/sec

static const float DT = 0.01;

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_POST_SCALE = 50;

static double runClimbRateSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = {
        CLIMBRATE_PRESCALE*setpoint,
        CLIMBRATE_PRESCALE*actual
    };

    vector <double> actions;
    snn->step(observations, actions);

    // NEGATE because our SNN used flip=true
    return -actions[0] * CLIMBRATE_KP;
}

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

static double runRollRateSnn(
        SNN * snn, const float setpoint, const float actual)
{
    vector<double> observations = { setpoint, actual };

    vector <double> actions;
    snn->step(observations, actions);

    // NEGATE because our SNN used flip=true
    return -actions[0] * PITCH_ROLL_POST_SCALE;
}

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

    hf::PitchRollAnglePid pitchRollAnglePid = {};
    hf::PitchRollRatePid pitchRollRatePid = {};

    SNN * climbrate_snn = NULL;
    SNN * yawrate_snn = NULL;
    SNN * roll_rate_snn = NULL;

    // Load up the network specified in the command line

    if (argc < 2) {
        fprintf(stderr, "Usage: %s RISP_NETWORK\n", argv[0]);
        exit(1);
    }

    try {

        climbrate_snn = new SNN(argv[1], "risp");
        yawrate_snn = new SNN(argv[1], "risp");
        roll_rate_snn = new SNN(argv[1], "risp");

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

        static bool ready;
        if (!ready) {
            printf("PID,SNN\n");
        }
        ready = true;

        // Get thrust demand from SNN
        const auto thrustFromSnn =
            runClimbRateSnn(climbrate_snn, sim.throttle(), sim.dz());

        // Get yaw demand from SNN
        const auto yawDemand = runYawRateSnn(yawrate_snn, sim.yaw(), sim.dpsi());

        // Use position PID controller to convert stick demands into pitch,roll angles
        float rollDemand = sim.roll();
        float pitchDemand  = sim.pitch();
        hf::PositionPid::run(rollDemand, pitchDemand, sim.dx(), sim.dy());

        const auto resetPids = sim.throttle() < THROTTLE_DOWN;

        // Use pitch,roll angle PID controllers to convert pitch,roll angles
        // into angular rates
        pitchRollAnglePid.run(DT, resetPids, rollDemand, pitchDemand,
                sim.phi(), sim.theta());

        // Use pitch,roll angle rate controllers to convert pitch,roll rates
        // into motor spins
        pitchRollRatePid.run( DT, resetPids, rollDemand, pitchDemand,
                sim.dphi(), sim.dtheta(), PITCH_ROLL_POST_SCALE);

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
