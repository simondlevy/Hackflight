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
#include <utils.hpp>

static const float THRUST_TAKEOFF = 56;

static const float THRUST_BASE = 55.2;

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    hf::Simulator sim = {};

    sim.init();

    while (true) {

        hf::state_t state = {};
        hf::demands_t stickDemands = {};
        bool hitTakeoffButton = false;
        bool completedTakeoff = false;

        if (!sim.step(stickDemands, state, hitTakeoffButton, completedTakeoff)) {
            break;
        }

        float thrust = 0;

        if (completedTakeoff) {

            thrust = THRUST_BASE - 25 * state.dz;
        }

        else if (hitTakeoffButton) {
            thrust = THRUST_TAKEOFF;
        }

        sim.setMotors(thrust, thrust, thrust, thrust);
    }

    wb_robot_cleanup();

    return 0;
}
