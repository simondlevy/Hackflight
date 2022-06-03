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
    static constexpr float Kp             = 0.75;
    static constexpr float Ki             = 1.5;
    static constexpr float PILOT_VELZ_MAX = 2.5;
    static constexpr float STICK_DEADBAND = 0.2;
    static constexpr float WINDUP_MAX     = 0.4;

    static bool _inBandPrev;
    static float _errorI;
    static float _altitudeTarget;

    bool didReset = false;

    float altitude = vstate->z;

    float throttle = 2 * demands->throttle - 1; // [0,1] => [-1,+1]

    // Is stick demand in deadband?
    bool inBand = fabs(throttle) < STICK_DEADBAND; 

    debugPrintf("throttle: %+3.3f   inband: %d", throttle, inBand);

    // Reset controller when moving into deadband
    if (reset || (inBand && !_inBandPrev)) {
        _errorI = 0;
        didReset = true;
    }
    _inBandPrev = inBand;

    // Target velocity is a setpoint inside deadband, scaled
    // constant outside
    float targetVelocity = inBand ?
        _altitudeTarget - altitude :
        PILOT_VELZ_MAX * throttle;

    // Compute error as scaled target minus actual
    float error = targetVelocity - vstate->dz;

    // Compute I term, avoiding windup
    _errorI = constrainAbs(_errorI + error, WINDUP_MAX);

    // Adjust throttle demand based on error
    //demands->throttle = error * Kp + _errorI * Ki;

    // If we re-entered deadband, we reset the target altitude.
    if (didReset) {
        _altitudeTarget = altitude;
    }
}
