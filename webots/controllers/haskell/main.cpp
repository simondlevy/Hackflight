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
#include <estimators/vertical.hpp>

// Webots simulator class
static hf::Simulator _sim;

// Global data and routines shared with Haskell Copilot ----------------------

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

bool stream_requestedTakeoff;

bool stream_completedTakeoff;

void debug(float value)
{
    printf("%+3.3f\n", value);
}

void copilot_step_core(void);

void setMotors(float m1, float m2, float m3, float m4)
{
    const float motors[4] = {m1, m2, m3, m4};

    _sim.setMotors(motors);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    _sim.init(false);

    while (true) {

        if (!_sim.step()) {
            break;
        }

        const auto demands = _sim.getDemandsFromKeyboard();

        stream_throttle = demands.thrust;
        stream_roll = demands.roll;
        stream_pitch = demands.pitch;
        stream_yaw = demands.yaw;

        stream_requestedTakeoff = _sim.requestedTakeoff();

        stream_completedTakeoff = _sim.time() > 3;

        const auto state = _sim.getState();

        stream_dx = state.dx;
        stream_dy = state.dy;
        stream_z = state.z;
        stream_dz = state.dz;
        stream_phi = state.phi;
        stream_dphi = state.dphi;
        stream_theta = state.theta;
        stream_dtheta = state.dtheta;
        stream_psi = state.psi;
        stream_dpsi = state.dpsi;

        copilot_step_core();
    }

    _sim.close();

    return 0;
}
