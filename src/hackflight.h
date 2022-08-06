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

#include "arming.h"
#include "datatypes.h"
#include "debug.h"
#include "deg2rad.h"
#include "failsafe.h"
#include "gyro.h"
#include "imu.h"
#include "maths.h"
#include "motor.h"
#include "pids/angle.h"
#include "rx.h"
#include "time.h"

// Scheduling constants -------------------------------------------------------

static const uint32_t RX_TASK_RATE       = 33;
static const uint32_t ATTITUDE_TASK_RATE = 100;

// PID-limiting constants -----------------------------------------------------

static const float    PID_MIXER_SCALING = 1000;
static const uint16_t PIDSUM_LIMIT_YAW  = 400;
static const uint16_t PIDSUM_LIMIT      = 500;


// PID controller support -----------------------------------------------------

static void hackflightAddPidController(hackflight_t * hf, pid_fun_t fun, void * data)
{
    hf->pidControllers[hf->pidCount].fun = fun;
    hf->pidControllers[hf->pidCount].data = data;
    hf->pidCount++;
}

// Core tasks: gyro, PID controllers, mixer, motors ---------------------------

static float constrain_demand(float demand, float limit, float scaling)
{
    return constrain_f(demand, -limit, +limit) / scaling;
}

// Public API -----------------------------------------------------------------

static void hackflightRunCoreTasks(
        hackflight_t * hf,
        uint32_t usec,
        bool failsafe,
        motor_config_t * motorConfig,
        float motorvals[])
{
    // Run PID controllers to get new demands
    for (uint8_t k=0; k<hf->pidCount; ++k) {
        pid_controller_t pid = hf->pidControllers[k];
        pid.fun(usec, &hf->demands, pid.data, &hf->vstate, hf->pidReset);
    }


    // Constrain the demands, negating yaw to make it agree with PID
    demands_t * demands = &hf->demands;
    demands->roll  =
        constrain_demand(demands->roll, PIDSUM_LIMIT, PID_MIXER_SCALING),
        demands->pitch =
            constrain_demand(demands->pitch, PIDSUM_LIMIT, PID_MIXER_SCALING),
        demands->yaw   =
            -constrain_demand(demands->yaw, PIDSUM_LIMIT_YAW, PID_MIXER_SCALING),

        // Run the mixer to get motors from demands
        hf->mixer(&hf->demands, failsafe, motorConfig, motorvals);
}

static void hackflightInit(
        hackflight_t * hf,
        anglePidConstants_t * anglePidConstants,
        mixer_t mixer)
{
    hf->mixer = mixer;

    anglePidInit(&hf->anglepid, anglePidConstants);

    hackflightAddPidController(hf, anglePidUpdate, &hf->anglepid);
}
