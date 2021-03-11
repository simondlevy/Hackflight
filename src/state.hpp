/*
   State datatype

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

namespace hf {

    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH, 
        AXIS_YAW
    };

    // See Bouabdallah et al. (2004)
    enum {
        STATE_X = 0,
        STATE_DX,
        STATE_Y,
        STATE_DY,
        STATE_Z,
        STATE_DZ,
        STATE_PHI,
        STATE_DPHI,
        STATE_THETA,
        STATE_DTHETA,
        STATE_PSI,
        STATE_DPSI,
        STATE_SIZE
    };

    typedef struct {

        float x[STATE_SIZE];

        bool armed;
        bool failsafe;

    } state_t;

} // namespace hf
