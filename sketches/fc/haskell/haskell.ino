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

#include <hackflight.hpp>
#include <board_mpu.hpp>
#include <receivers/sbus.hpp>

static const uint8_t LED_PIN = 0;

static hf::Board _board;

static hf::SbusReceiver _rx;

// Shared with Haskell Copilot -----------------------------------------------

float stream_dt;

bool stream_reset;

float stream_thro_demand;
float stream_roll_demand;
float stream_pitch_demand;
float stream_yaw_demand;

float stream_phi;
float stream_theta;

float stream_dphi;
float stream_dtheta;
float stream_dpsi;


void setMotors(float m1, float m2, float m3, float m4)
{
    const float motors[4] = {m1, m2, m3, m4};

    _board.runMotors(_rx, motors);
}

// ---------------------------------------------------------------------------

void setup() 
{
    _board.init(_rx);
}

void loop() 
{
    hf::demands_t demands = {};
    hf::state_t state = {};

    _board.readData(stream_dt, _rx, demands, state);

    stream_thro_demand = demands.thrust;
    stream_roll_demand = demands.roll;
    stream_pitch_demand = demands.pitch;
    stream_yaw_demand = demands.yaw;

    stream_phi = state.phi;
    stream_theta = state.theta;
    stream_dphi = state.dphi;
    stream_dtheta = state.dtheta;
    stream_dpsi = state.dpsi;

    stream_reset = stream_thro_demand < 0.06;

    // Run Haskell Copilot, which will call setMotors() above
    void copilot_step_core();
    copilot_step_core();
}
