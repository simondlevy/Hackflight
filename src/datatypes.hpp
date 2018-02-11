/*
    datatypes.hpp : Datatype declarations

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

#include <cstdint>

typedef struct {

    float throttle;
    float roll;
    float pitch;
    float yaw;

    uint8_t aux;

} demands_t;

typedef struct {

    float values[3];
    float derivs[3];

} stateval_t;

typedef struct {

    stateval_t position;    // X, Y, Z

    stateval_t orientation; // roll, pitch, yaw

    bool armed;

} vehicle_state_t;
