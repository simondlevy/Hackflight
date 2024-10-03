/*
  Hackflight example with Haskell PID controllers

  Adapted from https://github.com/nickrehm/dRehmFlight
 
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

#include <turtle_board_sbus.hpp>

static hf::TurtleBoardSbus _board;

// Shared with Haskell Copilot -----------------------------------------------

float stream_dt;
bool stream_reset;

float stream_thro_demand;
float stream_roll_demand;
float stream_pitch_demand;
float stream_yaw_demand;

float stream_roll_PID;
float stream_pitch_PID;
float stream_yaw_PID;

float stream_phi;
float stream_theta;

float stream_dphi;
float stream_dtheta;
float stream_dpsi;


void setMotors(float m1, float m2, float m3, float m4)
{
    _board.runMotors(m1, m2, m3, m4);
}

// ---------------------------------------------------------------------------

void setup() 
{
    _board.init();
}

void loop() 
{
    float psi = 0; // unused

    _board.readData(stream_dt, 
            stream_thro_demand,
            stream_roll_demand,
            stream_pitch_demand,
            stream_yaw_demand,
            stream_phi, stream_theta, psi, 
            stream_dphi, stream_dtheta, stream_dpsi);

    stream_reset = stream_thro_demand < 0.06;

    // Run Haskell Copilot, which will call setMotors() above
    void copilot_step_core();
    copilot_step_core();
}
