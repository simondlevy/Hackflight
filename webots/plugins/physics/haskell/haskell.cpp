/* 
 * Custom physics plugin for Hackflight simulator using Haskell Copilot for
 * control
 *
 *  Copyright (C) 2024 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <sim/sensors/groundtruth.hpp>

// Global data and routines shared with Haskell Copilot ----------------------

float stream_time;

float stream_dt;

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

static hf::demands_t _demands;

bool stream_requestedTakeoff;

void setDemands(float t, float r, float p, float y)
{
    _demands.thrust = t;
    _demands.roll = r;
    _demands.pitch = p;
    _demands.yaw = y;
}

void copilot_step_core();

// ---------------------------------------------------------------------------

namespace hf {

    // Called by webots_physics_init(); unneeded here
    void setup_controllers()
    {
    }

    demands_t run_controllers(
            const float pid_dt,
            const siminfo_t & siminfo,
            const state_t & state)
    {    

        // Count elapsed time since takeoff, for climb-rate PID control
        static uint32_t _frame_count;
        stream_time = siminfo.requested_takeoff ?
            _frame_count++ / siminfo.framerate : 0;

        stream_requestedTakeoff = siminfo.requested_takeoff;

        stream_throttle = siminfo.demands.thrust;
        stream_roll = siminfo.demands.roll;
        stream_pitch = siminfo.demands.pitch;
        stream_yaw = siminfo.demands.yaw;

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

        stream_dt = pid_dt;

        // This will call setDemands() defined above
        copilot_step_core();

        return _demands;
    }

    // Called by webots_physics_step()
    state_t estimate_state(
            const Dynamics & dynamics, const float pid_rate)
    {
        (void)pid_rate;

        return GroundTruth::read(dynamics);
    }

}
