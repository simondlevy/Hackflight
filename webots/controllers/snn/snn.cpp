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
#include <pids/yaw_rate.hpp>

static const int VIZ_PORT = 8100;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

static const float TAKEOFF_TIME = 3;

static const float OBSERVATION_SCALEDOWN = 0.5;

static const float ACTION_SCALEUP = 25;

static const float DT = 0.01;

static const float YAW_PRESCALE = 160; // deg/sec

static const float THROTTLE_DOWN = 0.06;

static const float PITCH_ROLL_POST_SCALE = 50;

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

    hf::PitchRollAnglePid pitchRollAnglePid = {};
    hf::PitchRollRatePid pitchRollRatePid = {};

    hf::YawRatePid yawRatePid = {};


    SNN * snn = NULL;

    // Load up the network specified in the command line

    if (argc < 2) {
        fprintf(stderr, "Usage: %s RISP_NETWORK\n", argv[0]);
        exit(1);
    }

    try {

        snn = new SNN(argv[1], "risp");

    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    snn->serve_visualizer(VIZ_PORT);

    while (true) {

        if (!sim.step()) {
            break;
        }

        // Get thrust from SNN -----------------------------------------------

        vector<double> observations = {
            OBSERVATION_SCALEDOWN*sim.throttle(),
            OBSERVATION_SCALEDOWN*sim.dz()
        };

        vector <double> actions;
        snn->step(observations, actions);

        static bool ready;

        if (!ready) {
            printf("setpoint,actual,difference,action\n");
        }
        ready = true;

        printf("%f,%f,%f,%f\n",
                sim.throttle(),
                sim.dz(),
                sim.throttle() - sim.dz(),
                actions[0]);
        fflush(stdout);

        // Hack because we used flip=true
        actions[0] = -actions[0];

        actions[0] *= ACTION_SCALEUP;

        const auto time = sim.hitTakeoffButton() ? sim.time() : 0;

        // Get roll, pitch, yaw from traditional PID controllers

        float rollDemand = sim.roll();

        float pitchDemand  = sim.pitch();

        float yawDemand = sim.yaw() * YAW_PRESCALE;

        hf::PositionPid::run(rollDemand, pitchDemand, sim.dx(), sim.dy());

        const auto resetPids = sim.throttle() < THROTTLE_DOWN;

        pitchRollAnglePid.run(DT, resetPids, rollDemand, pitchDemand,
                sim.phi(), sim.theta());

        pitchRollRatePid.run( DT, resetPids, rollDemand, pitchDemand,
                sim.dphi(), sim.dtheta(), PITCH_ROLL_POST_SCALE);

        yawRatePid.run(DT, resetPids, yawDemand, sim.dpsi());


        const auto thrustDemand =
            time > TAKEOFF_TIME ? 
            THRUST_BASE + actions[0] :
            sim.hitTakeoffButton() ? 
            THRUST_TAKEOFF :
            0;

        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runBetaFlightQuadX(
                thrustDemand,
                rollDemand,
                pitchDemand,
                yawDemand,
                m1, m2, m3, m4);

        sim.setMotors(m1, m2, m3, m4);

        snn->send_counts_to_visualizer();
    }

    wb_robot_cleanup();

    return 0;
}
