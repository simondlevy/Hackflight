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
#include "maths.h"
#include "mixer.h"
#include "motor.h"
#include "pids/angle.h"
#include "quat2euler.h"
#include "rx.h"
#include "time.h"

// Arming safety  -------------------------------------------------------------

static const float MAX_ARMING_ANGLE = 25;

// Scheduling constants -------------------------------------------------------

static const uint32_t RX_TASK_RATE       = 33;
static const uint32_t ATTITUDE_TASK_RATE = 100;

// Attitude task --------------------------------------------------------------

static void task_attitude(void * hackflight, uint32_t time)
{
    hackflight_t * hf = (hackflight_t *)hackflight;
    imuGetEulerAngles(hf, time);
}

// PID controller support -----------------------------------------------------

static void hackflightAddPidController(hackflight_t * hf, pid_fun_t fun, void * data)
{
    hf->pid_controllers[hf->pid_count].fun = fun;
    hf->pid_controllers[hf->pid_count].data = data;
    hf->pid_count++;
}

// RX polling task ------------------------------------------------------------

static void task_rx(void * hackflight, uint32_t time)
{
    hackflight_t * hf = (hackflight_t *)hackflight;

    bool calibrating = hf->gyroIsCalibrating; // || acc.calibrating != 0;
    bool pidItermResetReady = false;
    bool pidItermResetValue = false;

    float max_arming_angle = deg2rad(MAX_ARMING_ANGLE);

    rx_axes_t rxax = {0};

    bool gotNewData = false;

    bool shallowAngle = fabsf(hf->vstate.phi) < max_arming_angle &&
        fabsf(hf->vstate.theta) < max_arming_angle;

    rxPoll(
            time,
            shallowAngle, 
            calibrating,
            &rxax,
            &pidItermResetReady,
            &pidItermResetValue,
            &hf->armed,
            &gotNewData);

    if (pidItermResetReady) {
        hf->pid_zero_throttle_iterm_reset = pidItermResetValue;
    }

    if (gotNewData) {
        memcpy(&hf->rx_axes, &rxax, sizeof(rx_axes_t));
    }
}

// Core tasks: gyro, PID controllers, mixer, motors ------------------------

static void hackflightRunCoreTasks(hackflight_t * hf)
{
    gyroReadScaled(hf);

    timeUs_t currentTimeUs = timeMicros();

    rxGetDemands(currentTimeUs, &hf->anglepid, &hf->demands);

    for (uint8_t k=0; k<hf->pid_count; ++k) {
        pid_controller_t pid = hf->pid_controllers[k];
        pid.fun(currentTimeUs, &hf->demands, pid.data,
                &hf->vstate, hf->pid_zero_throttle_iterm_reset);
    }

    float mixmotors[4] = {0};
    mixerRun(&hf->demands, mixmotors);

    motorWrite(hf->armed ? mixmotors : hf->mspmotors);
}

// Timed task support -------------------------------------------------------

static void initTask(task_t * task, task_fun_t fun, uint32_t rate)
{
    task->fun = fun;
    task->desiredPeriodUs = 1000000 / rate;
}

// Sensor support ------------------------------------------------------------

static void hackflightAddSensor(hackflight_t * hf, task_fun_t fun, uint32_t rate)
{
    initTask(&hf->sensor_tasks[hf->sensor_task_count++], fun, rate);
}

// Initialization -------------------------------------------------------------

static void hackflightInit(
        hackflight_t * hf,
        float rate_p,
        float rate_i,
        float rate_d,
        float rate_f,
        float level_p
        )
{
    anglePidInit(&hf->anglepid, rate_p, rate_i, rate_d, rate_f, level_p);

    hackflightAddPidController(hf, anglePidUpdate, &hf->anglepid);

    initTask(&hf->attitudeTask, task_attitude, ATTITUDE_TASK_RATE);

    initTask(&hf->rxTask, task_rx,  RX_TASK_RATE);

    rxDevInit();
}

// Hardare version -------------------------------------------------------------

void hackflightFullInit(
        hackflight_t * hackflight,
        task_fun_t accelFun,
        uint32_t accel_rate);

void hackflightStep(hackflight_t * hackflight);


