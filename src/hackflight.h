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

#include "arming.h"
#include "datatypes.h"
#include "debug.h"
#include "deg2rad.h"
#include "failsafe.h"
#include "gyro.h"
#include "imu.h"
#include "init_task.h"
#include "maths.h"
#include "mixer.h"
#include "motor.h"
#include "pids/angle.h"
#include "rx.h"
#include "time.h"

// Arming safety  -------------------------------------------------------------

static const float MAX_ARMING_ANGLE = 25;

// Scheduling constants -------------------------------------------------------

static const uint32_t RX_TASK_RATE       = 33;
static const uint32_t ATTITUDE_TASK_RATE = 100;

// Attitude task --------------------------------------------------------------

#if defined(__cplusplus)
extern "C" {
#endif

static void task_attitude(void * hackflight, uint32_t time)
{
    hackflight_t * hf = (hackflight_t *)hackflight;
    imuGetEulerAngles(hf, time);
}

// PID controller support -----------------------------------------------------

static void hackflightAddPidController(hackflight_t * hf, pid_fun_t fun, void * data)
{
    hf->pidControllers[hf->pidCount].fun = fun;
    hf->pidControllers[hf->pidCount].data = data;
    hf->pidCount++;
}

// RX polling task ------------------------------------------------------------

static void task_rx(void * hackflight, uint32_t time)
{
    hackflight_t * hf = (hackflight_t *)hackflight;

    bool calibrating = hf->gyro.isCalibrating; // || acc.calibrating != 0;
    bool pidItermResetReady = false;
    bool pidItermResetValue = false;

    rx_axes_t rxax = {0};

    bool gotNewData = false;

    bool imuIsLevel =
        fabsf(hf->vstate.phi) < hf->maxArmingAngle &&
        fabsf(hf->vstate.theta) < hf->maxArmingAngle;

    rxPoll(
            &hf->rx,
            time,
            imuIsLevel, 
            calibrating,
            &rxax,
            hf->motorDevice,
            &hf->arming,
            &pidItermResetReady,
            &pidItermResetValue,
            &gotNewData);

    if (pidItermResetReady) {
        hf->pidZeroThrottleItermReset = pidItermResetValue;
    }

    if (gotNewData) {
        memcpy(&hf->rxAxes, &rxax, sizeof(rx_axes_t));
    }
}

// Core tasks: gyro, PID controllers, mixer, motors ------------------------

static void hackflightRunCoreTasks(hackflight_t * hf)
{
    gyroReadScaled(&hf->gyro, &hf->vstate);

    uint32_t currentTimeUs = timeMicros();

    rxGetDemands(&hf->rx, currentTimeUs, &hf->anglepid, &hf->demands);

    for (uint8_t k=0; k<hf->pidCount; ++k) {
        pid_controller_t pid = hf->pidControllers[k];
        pid.fun(currentTimeUs, &hf->demands, pid.data,
                &hf->vstate, hf->pidZeroThrottleItermReset);
    }

    float mixmotors[4] = {0};
    mixerRun(hf->mixer, &hf->demands, mixmotors);

    motorWrite(hf->motorDevice,
            armingIsArmed(&hf->arming) ? mixmotors : hf->mspMotors);
}

// Initialization -------------------------------------------------------------

static void hackflightInit(
        hackflight_t * hf,
        mixer_t mixer,
        serialPortIdentifier_e rxPort,
        float rateP,
        float rateI,
        float rateD,
        float rateF,
        float levelP
        )
{
    hf->mixer = mixer;

    anglePidInit(&hf->anglepid, rateP, rateI, rateD, rateF, levelP);

    hackflightAddPidController(hf, anglePidUpdate, &hf->anglepid);

    initTask(&hf->attitudeTask, task_attitude, ATTITUDE_TASK_RATE);

    initTask(&hf->rxTask, task_rx,  RX_TASK_RATE);

    // Initialize quaternion in upright position
    hf->imuFusionPrev.quat.w = 1;

    rxDevInit(rxPort);

    hf->maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);
}

#if defined(__cplusplus)
}
#endif
