/*
   datatypes.hpp : Datatype declarations

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

    typedef struct {

        float throttle;
        float roll;
        float pitch;
        float yaw;

    } demands_t;

    typedef struct {

        bool  armed;

        float location[3];
        float rotation[3]; 
        float angularVel[3]; 
        float bodyAccel[3]; 
        float bodyVel[3]; 
        float inertialVel[3]; 

    } state_t;

}; // namespace hf
