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

#include <stdio.h>
#include <string.h>

#include <control.hpp>
#include <datatypes.h>

// Global data and routines shared with Haskell Copilot ----------------------

float stream_dt;

bool stream_hovering;

float stream_thrust;
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

float stream_landing_altitude_m;

static demands_t _demands;

void setDemands(float t, float r, float p, float y)
{
    _demands.thrust = t;
    _demands.roll = r;
    _demands.pitch = p;
    _demands.yaw = y;
}

void copilot_step_core();

// ---------------------------------------------------------------------------

void runClosedLoopControl(
        const float dt,
        const bool inHoverMode,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    stream_dt = dt;

    stream_hovering = inHoverMode;

    stream_thrust = openLoopDemands.thrust;
    stream_roll = openLoopDemands.roll;
    stream_pitch = openLoopDemands.pitch;
    stream_yaw = openLoopDemands.yaw;

    stream_dx = vehicleState.dx;
    stream_dy = vehicleState.dy;
    stream_z = vehicleState.z;
    stream_dz = vehicleState.dz;
    stream_phi = vehicleState.phi;
    stream_dphi = vehicleState.dphi;
    stream_theta = vehicleState.theta;
    stream_dtheta = vehicleState.dtheta;
    stream_psi = vehicleState.psi;
    stream_dpsi = vehicleState.dpsi;

    stream_landing_altitude_m = landingAltitudeMeters;

    // This will call setDemands() defined above
    copilot_step_core();

    memcpy(&demands, &_demands, sizeof(demands_t));
}
