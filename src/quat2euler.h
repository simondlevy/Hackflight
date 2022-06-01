/*
Copyright (c) 2022 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include "datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

static void quat2euler(quaternion_t * quat, vehicle_state_t * state, rotation_t * rot)
{
    float qw = quat->w;
    float qx = quat->x;
    float qy = quat->y;
    float qz = quat->z;

    float r00 = 1 - 2 * qy*qy - 2 * qz*qz;
    float r10 = 2 * (qx*qy + qw*qz);
    float r20 = 2 * (qx*qz - qw*qy);
    float r21 = 2 * (qy*qz + qw*qx);
    float r22 = 1 - 2 * qx*qx - 2 * qy*qy;

    float psi = -atan2_approx(r10, r00); 

    // Results
    state->phi   = atan2_approx(r21, r22); 
    state->theta = (0.5f * M_PI) - acos_approx(-r20);
    state->psi   = psi + ((psi < 0) ? 2 * M_PI : 0);

    // Additional output
    rot->r20 = r20;
    rot->r21 = r21;
    rot->r22 = r22;
}

void alignSensorViaRotation(float *dest)
{
    const float x = dest[0];
    const float y = dest[1];
    const float z = dest[2];

    // 270 degrees
    dest[0] = -y;
    dest[1] = x;
    dest[2] = z;
}

#if defined(__cplusplus)
}
#endif
