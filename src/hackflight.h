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
#include "quat2euler.h"
#include "ratepid.h"
#include "rx.h"
#include "time.h"

// Task structure --------------------------------------------------------------

typedef struct {

    
    // For both hardware and sim implementations
    void (*fun)(uint32_t time);
    timeDelta_t desiredPeriodUs;        // target period of execution
    timeUs_t lastExecutedAtUs;          // last time of invocation

    // For hardware impelmentations
    uint16_t dynamicPriority;           // when last executed, to avoid task starvation
    uint16_t taskAgeCycles;
    timeUs_t lastSignaledAtUs;          // time of invocation event for event-driven tasks
    timeUs_t anticipatedExecutionTime;  // Fixed point expectation of next execution time

} task_t;

// Arming safety  -------------------------------------------------------------

static const float MAX_ARMING_ANGLE = 25;

// Scheduling constants -------------------------------------------------------

static const uint32_t RX_TASK_RATE       = 33;
static const uint32_t ATTITUDE_TASK_RATE = 100;

// Data shared among tasks -----------------------------------------------------

static bool            _armed;
static bool            _gyro_is_calibrating;
static float           _mspmotors[4];
static bool            _pid_zero_throttle_iterm_reset;
static rate_pid_t      _ratepid;
static vehicle_state_t _state;


// Attitude task --------------------------------------------------------------

static void task_attitude(uint32_t time)
{
    (void)time;

    quaternion_t quat = {0};

    imuGetQuaternion(time, _armed, &quat);

    rotation_t rot = {0};

    quat2euler(&quat, &_state, &rot);

    imuUpdateFusion(time, &quat, &rot);
}

static task_t _attitudeTask;

// RX polling task ------------------------------------------------------------

static task_t    _rxTask;
static rx_axes_t _rx_axes;

static void task_rx(uint32_t time)
{
    bool calibrating = _gyro_is_calibrating; // || acc.calibrating != 0;
    bool pidItermResetReady = false;
    bool pidItermResetValue = false;

    float max_arming_angle = deg2rad(MAX_ARMING_ANGLE);

    rx_axes_t rxax = {0};

    bool gotNewData = false;

    bool shallowAngle = fabsf(_state.phi) < max_arming_angle &&
        fabsf(_state.theta) < max_arming_angle;

    rxPoll(
            time,
            shallowAngle, 
            calibrating,
            &rxax,
            &pidItermResetReady,
            &pidItermResetValue,
            &_armed,
            &gotNewData);

    if (pidItermResetReady) {
        _pid_zero_throttle_iterm_reset = pidItermResetValue;
    }

    if (gotNewData) {
        memcpy(&_rx_axes, &rxax, sizeof(rx_axes_t));
    }
}

// Task support ----------------------------------------------------------------

static task_t  _tasks[20];
static uint8_t _task_count;

// Core tasks: gyro, rate PID, mixer, motors ----------------------------------

static void hackflightRunCoreTasks(void)
{
    gyroReadScaled(&_state, &_gyro_is_calibrating);

    timeUs_t currentTimeUs = timeMicros();

    demands_t demands = {0};
    rxGetDemands(currentTimeUs, &_ratepid, &demands);

    demands_t new_demands = {0};
    ratePidUpdate(
            currentTimeUs,
            &_ratepid,
            &demands,
            _pid_zero_throttle_iterm_reset,
            &_state,
            &new_demands);

    float mixmotors[4] = {0};
    mixerRun(&new_demands, mixmotors);

    motorWrite(_armed ? mixmotors : _mspmotors);
}

// General task support -------------------------------------------------------

static void initTask(task_t * task, void (*fun)(uint32_t time), uint32_t rate)
{
    task->fun = fun;
    task->desiredPeriodUs = 1000000 / rate;
}

static void hackflightAddTask(void (*fun)(uint32_t time), uint32_t rate)
{
    initTask(&_tasks[_task_count++], fun, rate);
}

// Initialization -------------------------------------------------------------

static void hackflightInit(void)
{
    ratePidInit(&_ratepid);

    initTask(&_attitudeTask, task_attitude, ATTITUDE_TASK_RATE);
    initTask(&_rxTask,  task_rx,  RX_TASK_RATE);

    rxDevInit();
}
