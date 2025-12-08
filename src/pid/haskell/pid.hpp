/* 
 *  Copyright (C) 2025 Simon D. Levy
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

#pragma once

#include <string.h>

#include <datatypes.h>
#include <serializer.hpp>

static demands_t _demands;

#ifdef _MAIN
void setDemands(float t, float r, float p, float y)
{
    _demands.thrust = t;
    _demands.roll = r;
    _demands.pitch = p;
    _demands.yaw = y;
}
#endif

void copilot_step_core();


class PidControl {

    public:

        void run(
                const float dt,
                const flightMode_t flightMode,
                const vehicleState_t & vehicleState,
                const demands_t & setpointDemands,
                demands_t & demands)
        {
            extern float stream_dt;

            extern bool stream_controlled;

            extern float stream_thrust;
            extern float stream_roll;
            extern float stream_pitch;
            extern float stream_yaw;

            extern float stream_dx;
            extern float stream_dy;
            extern float stream_z;
            extern float stream_dz;
            extern float stream_phi;
            extern float stream_dphi;
            extern float stream_theta;
            extern float stream_dtheta;
            extern float stream_psi;
            extern float stream_dpsi;

            stream_dt = dt;

            stream_controlled = flightMode == MODE_HOVERING || 
                flightMode == MODE_AUTONOMOUS;

            stream_thrust = setpointDemands.thrust;
            stream_roll = setpointDemands.roll;
            stream_pitch = setpointDemands.pitch;
            stream_yaw = setpointDemands.yaw;

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

            // This will call setDemands() defined above
            copilot_step_core();

            memcpy(&demands, &_demands, sizeof(demands_t));
        }

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        void init()
        {
        }

};
