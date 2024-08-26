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

#include <hackflight.hpp>
#include <mixers.hpp>
#include <sim.hpp>
#include <snn.hpp>

#include <pids/angle.hpp>
#include <pids/climbrate.hpp>
#include <pids/position.hpp>

#include <pids/position.hpp>

//#include <snn.hpp>

static const int VIZ_PORT = 8100;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

static const float DT = 0.01;

static const float YAW_ANGLE_MAX = 200;

static const float PITCH_ROLL_DEMAND_POST_SCALE = 30; // deg

static const float YAW_DEMAND_PRE_SCALE = 160; // deg/sec


int main(int argc, char ** argv)
{
    // Create a simulator object for Webots functionality 

    hf::Simulator sim = {};

    sim.init();

    hf::AnglePid _anglePid = {};

    // Load up the network specified in the command line

    SNN * snn = NULL;


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

        const auto thrust_demand = hf::ClimbRatePid::run(
                sim.hitTakeoffButton(),
                sim.completedTakeoff(),
                sim.throttle(),
                sim.dz());

        float roll_demand = 0;
        float pitch_demand = 0;

        hf::PositionPid::run(sim.roll(), sim.pitch(), sim.dx(), sim.dy(),
                roll_demand, pitch_demand);

        auto yaw_demand = sim.yaw() * YAW_DEMAND_PRE_SCALE;

        _anglePid.run(DT,
                1.0, // fake throttle to max for now, so no PID integral reset
                roll_demand, pitch_demand, yaw_demand,
                sim.phi(), sim.theta(), sim.dphi(),
                sim.dtheta(), sim.dpsi(),
                roll_demand, pitch_demand, yaw_demand);

        roll_demand *= PITCH_ROLL_DEMAND_POST_SCALE;

        pitch_demand *= PITCH_ROLL_DEMAND_POST_SCALE;

        float m1=0, m2=0, m3=0, m4=0;
        hf::Mixer::runBetaFlightQuadX(
                thrust_demand, roll_demand, pitch_demand, yaw_demand,
                m1, m2, m3, m4);

 
        sim.setMotors(m1, m2, m3, m4);

        // snn->send_counts_to_visualizer();

    }

    wb_robot_cleanup();

    return 0;
}
