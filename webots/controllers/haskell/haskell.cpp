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

float stream_throttle;
float stream_roll;
float stream_pitch;
float stream_yaw;

float stream_dx;
float stream_dy;
float stream_z;
float stream_dz;
float stream_phi;
float stream_dphi;
float stream_theta;
float stream_dtheta;
float stream_psi;
float stream_dpsi;

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

    while (_sim.step()) {

        stream_throttle = _sim.throttle();
        stream_roll = _sim.roll();
        stream_pitch = _sim.pitch();
        stream_yaw = _sim.yaw();

        stream_hitTakeoffButton = _sim.hitTakeoffButton();
        stream_completedTakeoff = _sim.completedTakeoff();

        stream_dx = _sim.dx();
        stream_dy = _sim.dy();
        stream_z = _sim.z();
        stream_dz = _sim.dz();
        stream_phi = _sim.phi();
        stream_dphi = _sim.dphi();
        stream_theta = _sim.theta();
        stream_dtheta = _sim.dtheta();
        stream_psi = _sim.psi();
        stream_dpsi = _sim.dpsi();

        copilot_step_core();
    }

    _sim.close();

    return 0;
}

