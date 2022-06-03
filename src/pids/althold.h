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
    static constexpr float Kp             = 7.5e-2;
    static constexpr float Ki             = 0;
    static constexpr float PILOT_VELZ_MAX = 2.5;
    static constexpr float STICK_DEADBAND = 0.2;
    static constexpr float WINDUP_MAX     = 0.4;

    static bool _inBandPrev;
    static float _errorI;
    static float _altitudeTarget;

    bool gotNewTarget = false;

    // NED => ENU
    float altitude = -vstate->z;

    float throttle = 2 * demands->throttle - 1; // [0,1] => [-1,+1]

    // Is stick demand in deadband?
    bool inBand = fabs(throttle) < STICK_DEADBAND; 

    // Reset controller when moving into deadband
    if (inBand && !_inBandPrev) {
        _errorI = 0;
        gotNewTarget = true;
    }
    _inBandPrev = inBand;

    if (reset) {
        _errorI = 0;
        _altitudeTarget = 0;
    }

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
    demands->throttle = ((error * Kp + _errorI * Ki) + 1) / 2; // [-1,+1] => [0,1]

    debugPrintf("%f", demands->throttle);

    // If we re-entered deadband, we reset the target altitude.
    if (gotNewTarget) {
        _altitudeTarget = altitude;
    }
}
