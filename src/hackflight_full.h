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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "arming.h"
#include "debug.h"
#include "deg2rad.h"
#include "failsafe.h"
#include "gyro.h"
#include "hackflight_core.h"
#include "led.h"
#include "motor.h"
#include "msp.h"
#include "rx.h"
#include "system.h"
#include "task.h"

// Scheduling constants -------------------------------------------------------

// Wait at start of scheduler loop if gyroSampleTask is nearly due
static const uint32_t SCHED_START_LOOP_MIN_US = 1;   
static const uint32_t SCHED_START_LOOP_MAX_US = 12;

// Fraction of a us to reduce start loop wait
static const uint32_t SCHED_START_LOOP_DOWN_STEP = 50;  

// Fraction of a us to increase start loop wait
static const uint32_t SCHED_START_LOOP_UP_STEP = 1;   

// Add an amount to the estimate of a task duration
static const uint32_t TASK_GUARD_MARGIN_MIN_US = 3;   
static const uint32_t TASK_GUARD_MARGIN_MAX_US = 6;

// Fraction of a us to reduce task guard margin
static const uint32_t TASK_GUARD_MARGIN_DOWN_STEP = 50;  

// Fraction of a us to increase task guard margin
static const uint32_t TASK_GUARD_MARGIN_UP_STEP = 1;   

// Add a margin to the amount of time allowed for a check function to run
static const uint32_t CHECK_GUARD_MARGIN_US = 2 ;  

// Some tasks have occasional peaks in execution time so normal moving average
// duration estimation doesn't work Decay the estimated max task duration by
// 1/(1 << TASK_EXEC_TIME_SHIFT) on every invocation
static const uint32_t TASK_EXEC_TIME_SHIFT = 7;

// Make aged tasks more schedulable
static const uint32_t TASK_AGE_EXPEDITE_COUNT = 1;   

// By scaling their expected execution time
static const float TASK_AGE_EXPEDITE_SCALE = 0.9; 

// Gyro interrupt counts over which to measure loop time and skew
static const uint32_t CORE_RATE_COUNT = 25000;
static const uint32_t GYRO_LOCK_COUNT = 400;

// Scheduling ------------------------------------------------------------------

typedef struct {
    int32_t loopStartCycles;
    int32_t loopStartMinCycles;
    int32_t loopStartMaxCycles;
    uint32_t loopStartDeltaDownCycles;
    uint32_t loopStartDeltaUpCycles;

    int32_t taskGuardCycles;
    int32_t taskGuardMinCycles;
    int32_t taskGuardMaxCycles;
    uint32_t taskGuardDeltaDownCycles;
    uint32_t taskGuardDeltaUpCycles;

    int32_t desiredPeriodCycles;
    uint32_t lastTargetCycles;

    uint32_t nextTimingCycles;

    int32_t guardMargin;
    uint32_t clockRate;

} scheduler_t;

// Arming safety  -------------------------------------------------------------

static const float MAX_ARMING_ANGLE = 25;

// General task type ----------------------------------------------------------

typedef void (*task_fun_t)(void * hp, void * dp, uint32_t usec);

typedef struct {

    // For both hardware and sim implementations
    void (*fun)(void * hp, void * dp, uint32_t time);
    int32_t desiredPeriodUs;            
    uint32_t lastExecutedAtUs;          

    // For hardware impelmentations
    uint16_t dynamicPriority;          
    uint16_t taskAgeCycles;
    uint32_t lastSignaledAtUs;         
    uint32_t anticipatedExecutionTime;

} task_t;

// Full structure for running Hackflight --------------------------------------

typedef struct {

    hackflight_core_t core;

    imu_align_fun imuAlignFun;
    task_data_t   taskData;
    scheduler_t   scheduler;

    task_t attitudeTask;
    task_t mspTask;
    task_t rxTask;

} hackflight_full_t;

// Attitude task --------------------------------------------------------------

static void task_attitude(void * hp, void * dp, uint32_t usec)
{
    hackflight_core_t * core = (hackflight_core_t *)hp;
    task_data_t * td = (task_data_t *)dp;

    imuGetEulerAngles(
            &td->gyro,
            &td->imuFusionPrev,
            &td->arming,
            usec,
            &core->vstate);
}

// RX polling task ------------------------------------------------------------

static void task_rx(void * hp, void * dp, uint32_t usec)
{
    hackflight_core_t * core = (hackflight_core_t *)hp;
    task_data_t * td = (task_data_t *)dp;

    bool calibrating = td->gyro.isCalibrating; // || acc.calibrating != 0;
    bool pidItermResetReady = false;
    bool pidItermResetValue = false;

    rx_axes_t rxax = {};

    bool gotNewData = false;

    bool imuIsLevel =
        fabsf(core->vstate.phi) < td->maxArmingAngle &&
        fabsf(core->vstate.theta) < td->maxArmingAngle;

    rxPoll(
            &td->rx,
            usec,
            imuIsLevel, 
            calibrating,
            &rxax,
            td->motorDevice,
            &td->arming,
            &pidItermResetReady,
            &pidItermResetValue,
            &gotNewData);

    if (pidItermResetReady) {
        core->pidReset = pidItermResetValue;
    }

    if (gotNewData) {
        memcpy(&td->rxAxes, &rxax, sizeof(rx_axes_t));
    }
}

// MSP task ---------------------------------------------------------------------

static const uint32_t MSP_TASK_RATE = 100;

static void task_msp(void * hp, void * dp, uint32_t usec)
{
    hackflight_core_t * core = (hackflight_core_t *)hp;
    task_data_t * td = (task_data_t *)dp;

    (void)usec;

    mspUpdate(
            &core->vstate,
            &td->rxAxes,
            armingIsArmed(&td->arming),
            td->motorDevice,
            td->mspMotors);
}

// Support for dynamically scheduled tasks ---------------------------------

static void adjustDynamicPriority(task_t * task, uint32_t usec) 
{
    // Task is time-driven, dynamicPriority is last execution age (measured
    // in desiredPeriods). Task age is calculated from last execution.
    task->taskAgeCycles = (cmpTimeUs(usec, task->lastExecutedAtUs) /
            task->desiredPeriodUs);
    if (task->taskAgeCycles > 0) {
        task->dynamicPriority = 1 + task->taskAgeCycles;
    }
}

// Increase priority for RX task
static void adjustRxDynamicPriority(rx_t * rx, task_t * task,
        uint32_t usec) 
{
    if (task->dynamicPriority > 0) {
        task->taskAgeCycles = 1 + (cmpTimeUs(usec,
                    task->lastSignaledAtUs) / task->desiredPeriodUs);
        task->dynamicPriority = 1 + task->taskAgeCycles;
    } else  {
        if (rxCheck(rx, usec)) {
            task->lastSignaledAtUs = usec;
            task->taskAgeCycles = 1;
            task->dynamicPriority = 2;
        } else {
            task->taskAgeCycles = 0;
        }
    }
}

static void executeTask(
        hackflight_core_t * core,
        task_data_t * dt,
        task_t *task,
        uint32_t usec)
{
    task->lastExecutedAtUs = usec;
    task->dynamicPriority = 0;

    uint32_t time = timeMicros();
    task->fun(core, dt, usec);

    uint32_t taskExecutionTimeUs = timeMicros() - time;

    if (taskExecutionTimeUs >
            (task->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT)) {
        task->anticipatedExecutionTime =
            taskExecutionTimeUs << TASK_EXEC_TIME_SHIFT;
    } else if (task->anticipatedExecutionTime > 1) {
        // Slowly decay the max time
        task->anticipatedExecutionTime--;
    }
}

static void checkCoreTasks(
        hackflight_full_t * full,
        hackflight_core_t * core,
        scheduler_t * scheduler,
        task_data_t * td,
        int32_t loopRemainingCycles,
        uint32_t nowCycles,
        uint32_t nextTargetCycles)
{
    if (scheduler->loopStartCycles > scheduler->loopStartMinCycles) {
        scheduler->loopStartCycles -= scheduler->loopStartDeltaDownCycles;
    }

    while (loopRemainingCycles > 0) {
        nowCycles = systemGetCycleCounter();
        loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
    }

    gyroReadScaled(&td->gyro, full->imuAlignFun, &core->vstate);

    uint32_t usec = timeMicros();

    rxGetDemands(&td->rx, usec, &core->anglePid, &core->demands);

    float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

    motor_config_t motorConfig = {
        motorValueDisarmed(),
        motorValueHigh(),
        motorValueLow(),
        motorIsProtocolDshot()  
    };

    hackflightRunCoreTasks(
            core,
            usec,
            failsafeIsActive(),
            &motorConfig,
            mixmotors);

    motorWrite(td->motorDevice,
            armingIsArmed(&td->arming) ? mixmotors : td->mspMotors);

    // CPU busy
    if (cmpTimeCycles(scheduler->nextTimingCycles, nowCycles) < 0) {
        scheduler->nextTimingCycles += scheduler->clockRate;
    }
    scheduler->lastTargetCycles = nextTargetCycles;

    // Bring the scheduler into lock with the gyro Track the actual gyro
    // rate over given number of cycle times and set the expected timebase
    static uint32_t _terminalGyroRateCount;
    static int32_t _sampleRateStartCycles;

    if ((_terminalGyroRateCount == 0)) {
        _terminalGyroRateCount = gyroInterruptCount() + CORE_RATE_COUNT;
        _sampleRateStartCycles = nowCycles;
    }

    if (gyroInterruptCount() >= _terminalGyroRateCount) {
        // Calculate number of clock cycles on average between gyro
        // interrupts
        uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
        scheduler->desiredPeriodCycles = sampleCycles / CORE_RATE_COUNT;
        _sampleRateStartCycles = nowCycles;
        _terminalGyroRateCount += CORE_RATE_COUNT;
    }

    // Track actual gyro rate over given number of cycle times and remove
    // skew
    static uint32_t _terminalGyroLockCount;
    static int32_t _gyroSkewAccum;

    int32_t gyroSkew =
        gyroGetSkew(nextTargetCycles, scheduler->desiredPeriodCycles);

    _gyroSkewAccum += gyroSkew;

    if ((_terminalGyroLockCount == 0)) {
        _terminalGyroLockCount = gyroInterruptCount() + GYRO_LOCK_COUNT;
    }

    if (gyroInterruptCount() >= _terminalGyroLockCount) {
        _terminalGyroLockCount += GYRO_LOCK_COUNT;

        // Move the desired start time of the gyroSampleTask
        scheduler->lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

        _gyroSkewAccum = 0;
    }
}

static void updateDynamicTask(
        task_t * task,
        task_t ** selected,
        uint16_t * selectedPriority)
{
    if (task->dynamicPriority > *selectedPriority) {
        *selectedPriority = task->dynamicPriority;
        *selected = task;
    }
}

static void adjustAndUpdateTask(
        task_t * task,
        uint32_t usec,
        task_t ** selectedTask,
        uint16_t * selectedTaskDynamicPriority)
{
    adjustDynamicPriority(task, usec);
    updateDynamicTask(task, selectedTask, selectedTaskDynamicPriority);
}

static void checkDynamicTasks(
        hackflight_full_t * full,
        hackflight_core_t * core,
        scheduler_t * scheduler,
        task_data_t * dt,
        int32_t loopRemainingCycles,
        uint32_t nextTargetCycles)
{
    task_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;

    uint32_t usec = timeMicros();

    adjustRxDynamicPriority(&dt->rx, &full->rxTask, usec);
    updateDynamicTask(&full->rxTask, &selectedTask,
            &selectedTaskDynamicPriority);

    adjustAndUpdateTask(&full->attitudeTask, usec,
            &selectedTask, &selectedTaskDynamicPriority);

    adjustAndUpdateTask(&full->mspTask, usec,
            &selectedTask, &selectedTaskDynamicPriority);

    if (selectedTask) {

        int32_t taskRequiredTimeUs =
            selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
        int32_t taskRequiredTimeCycles =
            (int32_t)systemClockMicrosToCycles((uint32_t)taskRequiredTimeUs);

        uint32_t nowCycles = systemGetCycleCounter();
        loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        // Allow a little extra time
        taskRequiredTimeCycles += scheduler->taskGuardCycles;

        if (taskRequiredTimeCycles < loopRemainingCycles) {
            uint32_t antipatedEndCycles =
                nowCycles + taskRequiredTimeCycles;
            executeTask(core, dt, selectedTask, usec);
            nowCycles = systemGetCycleCounter();
            int32_t cyclesOverdue =
                cmpTimeCycles(nowCycles, antipatedEndCycles);

            if ((cyclesOverdue > 0) ||
                    (-cyclesOverdue < scheduler->taskGuardMinCycles)) {
                if (scheduler->taskGuardCycles <
                        scheduler->taskGuardMaxCycles) {
                    scheduler->taskGuardCycles +=
                        scheduler->taskGuardDeltaUpCycles;
                }
            } else if (scheduler->taskGuardCycles >
                    scheduler->taskGuardMinCycles) {
                scheduler->taskGuardCycles -=
                    scheduler->taskGuardDeltaDownCycles;
            }
        } else if (selectedTask->taskAgeCycles > TASK_AGE_EXPEDITE_COUNT) {
            // If a task has been unable to run, then reduce it's recorded
            // estimated run time to ensure it's ultimate scheduling
            selectedTask->anticipatedExecutionTime *= 
                TASK_AGE_EXPEDITE_SCALE;
        }
    }
}

static void initTask(task_t * task, task_fun_t fun, uint32_t rate)
{
    task->fun = fun;
    task->desiredPeriodUs = 1000000 / rate;
}

static void schedulerInit(scheduler_t * scheduler)
{
    scheduler->loopStartCycles =
        systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
    scheduler->loopStartMinCycles =
        systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
    scheduler->loopStartMaxCycles =
        systemClockMicrosToCycles(SCHED_START_LOOP_MAX_US);
    scheduler->loopStartDeltaDownCycles =
        systemClockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
    scheduler->loopStartDeltaUpCycles =
        systemClockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

    scheduler->taskGuardMinCycles =
        systemClockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
    scheduler->taskGuardMaxCycles =
        systemClockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
    scheduler->taskGuardCycles = scheduler->taskGuardMinCycles;
    scheduler->taskGuardDeltaDownCycles =
        systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
    scheduler->taskGuardDeltaUpCycles =
        systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

    scheduler->lastTargetCycles = systemGetCycleCounter();

    scheduler->nextTimingCycles = scheduler->lastTargetCycles;

    scheduler->desiredPeriodCycles =
        (int32_t)systemClockMicrosToCycles(CORE_PERIOD());

    scheduler->guardMargin =
        (int32_t)systemClockMicrosToCycles(CHECK_GUARD_MARGIN_US);

    scheduler->clockRate = systemClockMicrosToCycles(1000000);
}

// -------------------------------------------------------------------------

void hackflightInitFull(
        hackflight_full_t * full,
        rx_dev_funs_t * rxDeviceFuns,
        serialPortIdentifier_e rxDevPort,
        anglePidConstants_t * anglePidConstants,
        mixer_t mixer,
        void * motorDevice,
        uint8_t imuInterruptPin,
        imu_align_fun imuAlign,
        uint8_t ledPin)
{
    hackflight_core_t * core = &full->core;

    hackflightInit(core, anglePidConstants, mixer);

    task_data_t * td = &full->taskData;

    mspInit();
    gyroInit(&td->gyro);
    imuInit(imuInterruptPin);
    ledInit(ledPin);
    ledFlash(10, 50);
    failsafeInit();
    failsafeReset();

    td->rx.devCheck = rxDeviceFuns->check;
    td->rx.devConvert = rxDeviceFuns->convert;

    rxDeviceFuns->init(rxDevPort);

    full->imuAlignFun = imuAlign;

    td->motorDevice = motorDevice;

    initTask(&full->attitudeTask, task_attitude, ATTITUDE_TASK_RATE);

    initTask(&full->rxTask, task_rx,  RX_TASK_RATE);

    // Initialize quaternion in upright position
    td->imuFusionPrev.quat.w = 1;

    td->maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);

    initTask(&full->mspTask, task_msp, MSP_TASK_RATE);

    schedulerInit(&full->scheduler);
}

void hackflightStep(hackflight_full_t * full)
{
    hackflight_core_t * core = &full->core;

    scheduler_t * scheduler = &full->scheduler;

    task_data_t * td = &full->taskData;

    uint32_t nextTargetCycles =
        scheduler->lastTargetCycles + scheduler->desiredPeriodCycles;

    // Realtime gyro/filtering/PID tasks get complete priority
    uint32_t nowCycles = systemGetCycleCounter();

    int32_t loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

    if (loopRemainingCycles < -scheduler->desiredPeriodCycles) {
        // A task has so grossly overrun that at entire gyro cycle has been
        // skipped This is most likely to occur when connected to the
        // configurator via USB as the serial task is non-deterministic
        // Recover as best we can, advancing scheduling by a whole number
        // of cycles
        nextTargetCycles += scheduler->desiredPeriodCycles * (1 +
                (loopRemainingCycles / -scheduler->desiredPeriodCycles));
        loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
    }

    // Tune out the time lost between completing the last task execution and
    // re-entering the scheduler
    if ((loopRemainingCycles < scheduler->loopStartMinCycles) &&
            (scheduler->loopStartCycles < scheduler->loopStartMaxCycles)) {
        scheduler->loopStartCycles += scheduler->loopStartDeltaUpCycles;
    }

    // Once close to the timing boundary, poll for its arrival
    if (loopRemainingCycles < scheduler->loopStartCycles) {
        checkCoreTasks(
                full,
                core,
                scheduler,
                td,
                loopRemainingCycles,
                nowCycles,
                nextTargetCycles);
    }

    int32_t newLoopRemainingCyles =
        cmpTimeCycles(nextTargetCycles, systemGetCycleCounter());

    if (newLoopRemainingCyles > scheduler->guardMargin) {
        checkDynamicTasks(
                full, core,
                scheduler,
                td,
                newLoopRemainingCyles,
                nextTargetCycles);
    }
}
