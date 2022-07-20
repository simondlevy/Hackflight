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
#include "debug.h"
#include "maths.h"
#include "pids/angle.h"
#include "time.h"

// PID-limiting constants -----------------------------------------------------

static const float    PID_MIXER_SCALING   = 1000;
static const uint16_t PIDSUM_LIMIT_CYCLIC = 500;
static const uint16_t PIDSUM_LIMIT_YAW    = 400;

// Attitude task --------------------------------------------------------------

#if defined(__cplusplus)
extern "C" {
#endif

// PID controller support -----------------------------------------------------

static void hackflightAddPidController(hackflight_t * hf, pid_fun_t fun, void * data)
{
    hf->pidControllers[hf->pidCount].fun = fun;
    hf->pidControllers[hf->pidCount].data = data;
    hf->pidCount++;
}

// Core tasks: gyro, PID controllers, mixer, motors ------------------------

static float constrain_demand(float demand, float limit)
{
    return constrain_f(demand, -limit, +limit) / PID_MIXER_SCALING;
}

static void hackflightStep(hackflight_t * hf, float mixmotors[])
{
    // Run PID controllers
    for (uint8_t k=0; k<hf->pidCount; ++k) {
        pidController_t pid = hf->pidControllers[k];
        pid.fun(timeMicros(), &hf->demands, pid.data,
                &hf->vstate, hf->pidZeroThrottleItermReset);
    }

    // Run mixer
    hf->mixer(
            hf->demands.throttle,
            constrain_demand(hf->demands.rpy.x, PIDSUM_LIMIT_CYCLIC),
            constrain_demand(hf->demands.rpy.y, PIDSUM_LIMIT_CYCLIC),
            -constrain_demand(hf->demands.rpy.z, PIDSUM_LIMIT_YAW),
            mixmotors);
}

// ============================================================================

static void hackflightInit(
        hackflight_t * hf,
        anglePidConstants_t * anglePidConstants,
        mixer_t mixer)
{
    hf->mixer = mixer;

    anglePidInit(&hf->anglepid, anglePidConstants);

    hackflightAddPidController(hf, anglePidUpdate, &hf->anglepid);
}

#if defined(__cplusplus)
}
#endif
