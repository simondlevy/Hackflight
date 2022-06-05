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
static demands_t       _demands;
static bool            _gyro_is_calibrating;
static float           _mspmotors[4];
static bool            _pid_zero_throttle_iterm_reset;
static rate_pid_t      _ratepid;
static vehicle_state_t _state;

// Attitude task --------------------------------------------------------------

static void task_attitude(uint32_t time)
{
    imuGetEulerAngles(time, &_state, _armed);
}

static task_t _attitudeTask;

// PID controller support -----------------------------------------------------

static pid_controller_t _pid_controllers[10];
static uint8_t          _pid_count;

static void hackflightAddPidController(pid_fun_t fun, void * data)
{
    _pid_controllers[_pid_count].fun = fun;
    _pid_controllers[_pid_count].data = data;
    _pid_count++;
}

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

// Core tasks: gyro, PID controllers, mixer, motors ------------------------

static void hackflightRunCoreTasks(void)
{
    gyroReadScaled(&_state, &_gyro_is_calibrating);

    timeUs_t currentTimeUs = timeMicros();

    rxGetDemands(currentTimeUs, &_ratepid, &_demands);

    for (uint8_t k=0; k<_pid_count; ++k) {
        pid_controller_t pid = _pid_controllers[k];
        pid.fun(currentTimeUs, &_demands, pid.data,
                &_state, _pid_zero_throttle_iterm_reset);
    }

    float mixmotors[4] = {0};
    mixerRun(&_demands, mixmotors);

    motorWrite(_armed ? mixmotors : _mspmotors);
}

// Timed task support -------------------------------------------------------

static void initTask(task_t * task, void (*fun)(uint32_t time), uint32_t rate)
{
    task->fun = fun;
    task->desiredPeriodUs = 1000000 / rate;
}

// Sensor support ------------------------------------------------------------

static task_t  _sensor_tasks[20];
static uint8_t _sensor_task_count;

static void hackflightAddSensor(void (*fun)(uint32_t time), uint32_t rate)
{
    initTask(&_sensor_tasks[_sensor_task_count++], fun, rate);
}

// Initialization -------------------------------------------------------------

static void hackflightInit(void)
{
    ratePidInit(&_ratepid);

    hackflightAddPidController(ratePidUpdate, &_ratepid);

    initTask(&_attitudeTask, task_attitude, ATTITUDE_TASK_RATE);
    initTask(&_rxTask,  task_rx,  RX_TASK_RATE);

    rxDevInit();
}
