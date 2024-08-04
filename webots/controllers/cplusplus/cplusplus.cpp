/*
  C++ flight simulator support for Hackflight
 
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

#include <tasks/core.hpp>
#include <webots.hpp>

static const float PITCH_ROLL_ANGLE_KP = 6e0;

static const float PITCH_ROLL_RATE_KP = 1.25e-2;
static const float PITCH_ROLL_RATE_KD = 0; //1.25e-4;

static const float YAW_RATE_KP = 1.20e-2;

// Motor thrust constants for climb-rate PID controller
static const float TBASE = 56;
static const float TSCALE = 0.25;
static const float TMIN = 0;

// Arbitrary time constexprant
static constexpr float DT = .01;

int main(int argc, char ** argv)
{
    hf::CoreTask coreTask = {};

    coreTask.init(
            PITCH_ROLL_ANGLE_KP, 
            PITCH_ROLL_RATE_KP, 
            PITCH_ROLL_RATE_KD,
            YAW_RATE_KP, 
            TBASE, 
            TSCALE, 
            TMIN,
            DT);

    hf::Simulator sim = {};

    sim.init();

    hf::state_t state = {};

    hf::demands_t demands = {};

    while (sim.step(demands, state)) {

        hf::quad_motors_t motors = {};

        coreTask.run(state, demands, motors);

        sim.setMotors(motors.m1, motors.m2, motors.m3, motors.m4);
    }

    sim.close();

    return 0;
}
