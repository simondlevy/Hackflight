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

static const float PITCH_ROLL_ANGLE_KP = 6;

static const float CLIMBRATE_KP = 4;
static const float CLIMBRATE_PRESCALE = 1.0;

static const float YAW_KP = 0.003;           
static const float YAW_PRESCALE = 160; // deg/sec

static const float DT = 0.01;

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_RATE_KP = 0.0125;

static const float POSITION_KP = 10;

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

static float runPositionPid(float setpoint, float actual)
{
    return POSITION_KP * (setpoint - actual);
}

static float runPitchRollAnglePid(float setpoint, float actual)
{
    return PITCH_ROLL_ANGLE_KP * (setpoint - actual);
}

static float runPitchRollRatePid(float setpoint, float actual)
{
    return PITCH_ROLL_RATE_KP * (setpoint - actual);
}

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

    SNN * climbrate_snn = NULL;
    SNN * yawrate_snn = NULL;

    // Load up the network specified in the command line

    if (argc < 2) {
        fprintf(stderr, "Usage: %s RISP_NETWORK\n", argv[0]);
        exit(1);
    }

    try {

        climbrate_snn = new SNN(argv[1], "risp");
        yawrate_snn = new SNN(argv[1], "risp");

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
        float rollDemand = runPositionPid(sim.roll(), sim.dy());
        float pitchDemand = runPositionPid(sim.pitch(), sim.dx());

        // Use pitch,roll angle PID controllers to convert pitch,roll angles
        // into angular rates
        rollDemand = runPitchRollAnglePid(rollDemand, sim.phi());
        pitchDemand = runPitchRollAnglePid(pitchDemand, sim.theta());

        // Use pitch,roll angle rate controllers to convert pitch,roll rates
        // into motor spins
        rollDemand = runPitchRollRatePid(rollDemand, sim.dphi());
        pitchDemand = runPitchRollRatePid(pitchDemand, sim.dtheta());

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
