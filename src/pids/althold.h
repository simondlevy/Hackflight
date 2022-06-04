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

void altHoldPidUpdate(
        uint32_t currentTimeUs
        , demands_t * demands
        , void * data
        , vehicle_state_t * vstate
        , bool reset
        )
{
    static constexpr float ALTITUDE_MIN   = 1.0;
    static constexpr float Kp             = 0.75;
    static constexpr float Ki             = 1.5;
    static constexpr float PILOT_VELZ_MAX = 2.5;
    static constexpr float STICK_DEADBAND = 0.2;
    static constexpr float WINDUP_MAX     = 0.4;

    static bool _inBandPrev;
    static float _errorI;
    static float _altitudeTarget;

    float  throttle = *(float *)data;

    // NED => ENU
    float altitude = vstate->z;

    float sthrottle = 2 * throttle - 1; // [0,1] => [-1,+1]

    // Is stick demand in deadband, above a minimum altitude?
    bool inBand = fabs(sthrottle) < STICK_DEADBAND && altitude > ALTITUDE_MIN; 

    // Reset controller when moving into deadband above a minimum altitude
    bool gotNewTarget = inBand && !_inBandPrev;
    _errorI = gotNewTarget || reset ? 0 : _errorI;
        
    _inBandPrev = inBand;

    if (reset) {
        _altitudeTarget = 0;
    }

    _altitudeTarget = gotNewTarget ? altitude : _altitudeTarget;

    // Target velocity is a setpoint inside deadband, scaled
    // constant outside
    float targetVelocity = inBand ?
        _altitudeTarget - altitude :
        PILOT_VELZ_MAX * sthrottle;

    // Compute error as scaled target minus actual
    float error = targetVelocity - vstate->dz;

    // Compute I term, avoiding windup
    _errorI = constrainAbs(_errorI + error, WINDUP_MAX);

    float correction = error * Kp + _errorI * Ki;

    // Adjust throttle demand based on error
    demands->throttle += correction;
}
