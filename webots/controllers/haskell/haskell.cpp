/*
  Haskell Copilot simulator support for Hackflight
 
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
#include <sim.hpp>

// Webots simulator class
static hf::Simulator _sim;

// Global data and routines shared with Haskell Copilot ----------------------

hf::state_t stream_vehicleState;

float stream_throttleStick;
float stream_rollStick;
float stream_pitchStick;
float stream_yawStick;

bool stream_hitTakeoffButton;

bool stream_completedTakeoff;

void debug(float value)
{
    printf("%+3.3f\n", value);
}

void copilot_step_core(void);

void setMotors(float m1, float m2, float m3, float m4)
{
    _sim.setMotors(m1, m2, m3, m4);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    _sim.init();

    while (_sim.step(
                stream_throttleStick, 
                stream_rollStick, 
                stream_pitchStick, 
                stream_yawStick, 
                stream_vehicleState,
                stream_hitTakeoffButton, 
                stream_completedTakeoff)) {

        copilot_step_core();
    }

    _sim.close();

    return 0;
}

