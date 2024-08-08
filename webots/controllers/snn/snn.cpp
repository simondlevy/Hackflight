/*
   C++ flight simulator spiking-neural net controller for Hackflight

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

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>

#include <webots.hpp>
#include <snn.hpp>
#include <utils.hpp>

static const bool USE_NETWORK = true;

// Small scaling value relating climb-rate demand to throttle stick
static const float THROTTLE_SCALE = 0.2;

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.385;

static const float TAKEOFF_TIME = 2;

// Non-neuro Utils::pcontrol(ler ---------------

static const float K_CLIMBRATE = 25;

// ------------------------------------

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    SNN * snn = NULL;

    try {
        snn = new SNN("networks/hover_risp.txt", "risp");
    } catch (const SRE &e) {
        fprintf(stderr, "Couldn't set up SNN:\n%s\n", e.what());
        exit(1);
    }

    const int timestep = (int)wb_robot_get_basic_time_step();

    uint32_t tick = 0;

    bool reached_altitude = false;

    bool button_was_hit = false;

    while (true) {

        hf::state_t state = {};

        hf::demands_t stickDemands = {};

        bool button = false;

        if (!sim.step(stickDemands, button, state)) {
            break;
        }

        if (button) {
            button_was_hit = true;
        }

        const auto scaledThrottle = THROTTLE_SCALE * stickDemands.thrust;

        // Get current climb rate observation
        const auto dz = state.dz;

        const double time = button_was_hit ? tick++ * timestep / 1000 : 0;

        if (time > TAKEOFF_TIME) {
            reached_altitude = true;
        }

        (void)snn;
        /*
        vector<double> o = {_ztarget, z, dz};
        vector <double> a;
        snn->getActions(o, a);
        const auto motor_snn = a[0];
        (void)motor_snn;*/

        const auto dz_target = scaledThrottle;

        const auto thrust = K_CLIMBRATE *  (dz_target - dz);

        const auto motor = reached_altitude  ? THRUST_BASE + thrust : 
            button_was_hit ? THRUST_TAKEOFF :
            0;

        sim.setMotors(motor, motor, motor, motor);
    }

    wb_robot_cleanup();

    return 0;
}
