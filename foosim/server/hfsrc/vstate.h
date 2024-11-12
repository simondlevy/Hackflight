/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

class VehicleState {

    public:

        float x;
        float dx;
        float y;
        float dy;
        float z;
        float dz;
        float phi;
        float dphi;
        float theta;
        float dtheta;
        float psi;
        float dpsi;

        VehicleState(
                float _x,
                float _dx,
                float _y,
                float _dy,
                float _z,
                float _dz,
                float _phi,
                float _dphi,
                float _theta,
                float _dtheta,
                float _psi,
                float _dpsi
             )

        {
            x = _x;
            dx = _dx;
            y = _y;
            dy = _dy;
            z = _z;
            dz = _dz;
            phi = _phi;
            dphi = _dphi;
            theta = _theta;
            dtheta = _dtheta;
            psi = _psi;
            dpsi = _dpsi;
        }

        VehicleState(void)
            : VehicleState(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        {
        }

        VehicleState(const VehicleState & state)
        {
            x = state.x;
            dx = state.dx;
            y = state.y;
            dy = state.dy;
            z = state.z;
            dz = state.dz;
            phi = state.phi;
            dphi = state.dphi;
            theta = state.theta;
            dtheta = state.dtheta;
            psi = state.psi;
            dpsi = state.dpsi;
        }
};
