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

#include <sim.hpp>

#include <levy_snn_util.hpp>

static const int VIZ_PORT = 8100;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

static const float TAKEOFF_TIME = 3;

static const float ACTION_SCALEUP = 12.5;

int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

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

        static bool ready;

        if (!ready) {
            printf("setpoint,actual,difference,action\n");
        }
        ready = true;


        vector<double> observations = {sim.throttle(), sim.dz()};
        vector <double> actions;
        snn->step(observations, actions);

        printf("%f,%f,%f,%f\n",
                sim.throttle(),
                sim.dz(),
                sim.throttle() - sim.dz(),
                actions[0]);
        fflush(stdout);

        actions[0] *= ACTION_SCALEUP;

        const auto time = sim.hitTakeoffButton() ? sim.time() : 0;

        const auto motor =
            time > TAKEOFF_TIME ? 
            THRUST_BASE + actions[0] :
            sim.hitTakeoffButton() ? 
            THRUST_TAKEOFF :
            0;

        snn->send_counts_to_visualizer();

        sim.setMotors(motor, motor, motor, motor);
    }

    wb_robot_cleanup();

    return 0;
}
