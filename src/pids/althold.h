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

static bool inBand(float value, float band) 
{
    return value > -band && value < band;
}

static float constrainAbs(float v, float lim)
{
    return v < -lim ? -lim : v > +lim ? +lim : v;
}

void altHoldRunController(vehicle_state_t * vstate, demands_t * demands)
{
    static constexpr float Kp             = 0.75;
    static constexpr float Ki             = 1.5;
    static constexpr float PILOT_VELZ_MAX = 2.5;
    static constexpr float STICK_DEADBAND = 0.2;
    static constexpr float WINDUP_MAX     = 0.4;

    // Controller state
    static float _altitudeTarget;
    static float _errI;
    static float _throttleDemand;

    // NED => ENU
    float altitude = -vstate->z;

    // Scale throttle [0,1] => [-1,+1] to support deadband
    bool inband = inBand(2*demands->throttle-1, STICK_DEADBAND);

    debugPrintf("%d", inband);

    // Reset controller when moving into deadband
    float altitudeTarget = inband &&  !inBand(_throttleDemand, STICK_DEADBAND) ?
        altitude :
        _altitudeTarget;

    // Hold altitude in band; scale throttle out of band
    float targetVelocity = inband ?
        altitudeTarget - altitude :
        PILOT_VELZ_MAX * demands->throttle;

    // Compute error as altTarget velocity minus actual velocity, after
    // negating actual to accommodate NED
    float err = targetVelocity + vstate->dz;

    // Accumualte error integral
    float errI = constrainAbs(_errI + err, WINDUP_MAX);

    // Implement PI control
    //demands->throttle =  Kp * err + Ki * errI;

    // Maintain controller state for next iteration
    _errI = errI;
    _altitudeTarget = altitudeTarget;
    _throttleDemand = demands->throttle;
}
