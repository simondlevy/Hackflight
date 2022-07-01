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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "accel.h"
#include "arming.h"
#include "board.h"
#include "gyro.h"
#include "debug.h"
#include "hackflight.h"
#include "led.h"
#include "msp.h"
#include "rx.h"
#include "system.h"

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

// MSP task ---------------------------------------------------------------------

static const uint32_t MSP_TASK_RATE = 100;

#if defined(__cplusplus)
extern "C" {
#endif

    static void task_msp(void * hackflight, uint32_t time)
    {
        (void)time;

        hackflight_t * hf = (hackflight_t *)hackflight;
        mspUpdate(&hf->vstate, &hf->rxAxes, armingIsArmed(&hf->arming),
                hf->motorDevice, hf->mspMotors);
    }

    // Support for dynamically scheduled tasks ---------------------------------

    static int32_t taskNextStateTime;

    static void adjustDynamicPriority(task_t * task, uint32_t currentTimeUs) 
    {
        // Task is time-driven, dynamicPriority is last execution age (measured
        // in desiredPeriods). Task age is calculated from last execution.
        task->taskAgeCycles = (cmpTimeUs(currentTimeUs, task->lastExecutedAtUs) /
                task->desiredPeriodUs);
        if (task->taskAgeCycles > 0) {
            task->dynamicPriority = 1 + task->taskAgeCycles;
        }
    }

    // Increase priority for RX task
    static void adjustRxDynamicPriority(rx_t * rx, task_t * task,
            uint32_t currentTimeUs) 
    {
        if (task->dynamicPriority > 0) {
            task->taskAgeCycles = 1 + (cmpTimeUs(currentTimeUs,
                        task->lastSignaledAtUs) / task->desiredPeriodUs);
            task->dynamicPriority = 1 + task->taskAgeCycles;
        } else  {
            if (rxCheck(rx, currentTimeUs)) {
                task->lastSignaledAtUs = currentTimeUs;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 2;
            } else {
                task->taskAgeCycles = 0;
            }
        }
    }

    static void executeTask(hackflight_t * hf, task_t *task, uint32_t currentTimeUs)
    {
        task->lastExecutedAtUs = currentTimeUs;
        task->dynamicPriority = 0;

        uint32_t time = timeMicros();
        task->fun(hf, currentTimeUs);

        uint32_t taskExecutionTimeUs = timeMicros() - time;

        // Update estimate of expected task duration
        taskNextStateTime = -1;
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
            hackflight_t * hf,
            int32_t loopRemainingCycles,
            uint32_t nowCycles,
            uint32_t nextTargetCycles)
    {
        scheduler_t * scheduler = &hf->scheduler;

        if (scheduler->loopStartCycles > scheduler->loopStartMinCycles) {
            scheduler->loopStartCycles -= scheduler->loopStartDeltaDownCycles;
        }

        while (loopRemainingCycles > 0) {
            nowCycles = systemGetCycleCounter();
            loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

        hackflightRunCoreTasks(hf);

        // CPU busy
        if (cmpTimeCycles(scheduler->nextTimingCycles, nowCycles) < 0) {
            scheduler->nextTimingCycles += scheduler->clockRate;
        }
        scheduler->lastTargetCycles = nextTargetCycles;

        // Bring the scheduler into lock with the gyro
        // Track the actual gyro rate over given number of cycle times and set the
        // expected timebase
        static uint32_t _terminalGyroRateCount;
        static int32_t _sampleRateStartCycles;

        if ((_terminalGyroRateCount == 0)) {
            _terminalGyroRateCount = gyroInterruptCount() + CORE_RATE_COUNT;
            _sampleRateStartCycles = nowCycles;
        }

        if (gyroInterruptCount() >= _terminalGyroRateCount) {
            // Calculate number of clock cycles on average between gyro interrupts
            uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
            scheduler->desiredPeriodCycles = sampleCycles / CORE_RATE_COUNT;
            _sampleRateStartCycles = nowCycles;
            _terminalGyroRateCount += CORE_RATE_COUNT;
        }

        // Track actual gyro rate over given number of cycle times and remove skew
        static uint32_t _terminalGyroLockCount;
        static int32_t _gyroSkewAccum;

        int32_t gyroSkew =
            imuGetGyroSkew(nextTargetCycles, scheduler->desiredPeriodCycles);

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

    static void updateDynamicTask(task_t * task, task_t ** selected,
            uint16_t * selectedPriority)
    {
        if (task->dynamicPriority > *selectedPriority) {
            *selectedPriority = task->dynamicPriority;
            *selected = task;
        }
    }

    static void adjustAndUpdateTask(
            task_t * task,
            uint32_t currentTimeUs,
            task_t ** selectedTask,
            uint16_t * selectedTaskDynamicPriority)
    {
        adjustDynamicPriority(task, currentTimeUs);
        updateDynamicTask(task, selectedTask, selectedTaskDynamicPriority);
    }

    static void checkDynamicTasks(
            hackflight_t * hf,
            int32_t loopRemainingCycles,
            uint32_t nextTargetCycles)
    {
        task_t *selectedTask = NULL;
        uint16_t selectedTaskDynamicPriority = 0;

        uint32_t currentTimeUs = timeMicros();

        for (uint8_t k=0; k<hf->sensorTaskCount; ++k) {
            task_t * task = &hf->sensorTasks[k];
            adjustAndUpdateTask(task, currentTimeUs,&selectedTask,
                    &selectedTaskDynamicPriority);
        }

        adjustRxDynamicPriority(&hf->rx, &hf->rxTask, currentTimeUs);
        updateDynamicTask(&hf->rxTask, &selectedTask, &selectedTaskDynamicPriority);

        adjustAndUpdateTask(&hf->attitudeTask, currentTimeUs,
                &selectedTask, &selectedTaskDynamicPriority);

        adjustAndUpdateTask(&hf->mspTask, currentTimeUs,
                &selectedTask, &selectedTaskDynamicPriority);

        if (selectedTask) {

            int32_t taskRequiredTimeUs =
                selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
            int32_t taskRequiredTimeCycles =
                (int32_t)systemClockMicrosToCycles((uint32_t)taskRequiredTimeUs);

            uint32_t nowCycles = systemGetCycleCounter();
            loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            scheduler_t * scheduler = &hf->scheduler;

            // Allow a little extra time
            taskRequiredTimeCycles += scheduler->taskGuardCycles;

            if (taskRequiredTimeCycles < loopRemainingCycles) {
                uint32_t antipatedEndCycles = nowCycles + taskRequiredTimeCycles;
                executeTask(hf, selectedTask, currentTimeUs);
                nowCycles = systemGetCycleCounter();
                int32_t cyclesOverdue = cmpTimeCycles(nowCycles, antipatedEndCycles);

                if ((cyclesOverdue > 0) ||
                        (-cyclesOverdue < scheduler->taskGuardMinCycles)) {
                    if (scheduler->taskGuardCycles < scheduler->taskGuardMaxCycles) {
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
                selectedTask->anticipatedExecutionTime *= TASK_AGE_EXPEDITE_SCALE;
            }
        }
    }

    // ----------------------------------------------------------------------------

    void hackflightInitFull(
            hackflight_t * hf,
            mixer_t mixer,
            void * motorDevice,
            serialPortIdentifier_e rxPort,
            uint8_t imuInterruptPin,
            imu_align_fun imuAlign,
            uint8_t ledPin)
    {
        // Tuning constants for angle PID controller
        static const float RATE_P  = 1.441305;
        static const float RATE_I  = 19.55048;
        static const float RATE_D  = 0.021160;
        static const float RATE_F  = 0.0165048;
        static const float LEVEL_P = 0 /*3.0*/;

        mspInit();
        gyroInit(hf);
        imuInit(hf, imuInterruptPin);
        ledInit(ledPin);
        ledFlash(10, 50);
        failsafeInit();
        failsafeReset();

        hf->imuAlignFun = imuAlign;

        hf->motorDevice = motorDevice;

        hackflightInit(hf, mixer, rxPort, RATE_P, RATE_I, RATE_D, RATE_F, LEVEL_P);

        hackflightAddSensor(hf, imuAccelTask, ACCEL_RATE);

        initTask(&hf->mspTask, task_msp, MSP_TASK_RATE);

        scheduler_t * scheduler = &hf->scheduler;

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

    void hackflightStep(hackflight_t * hf)
    {
        scheduler_t * scheduler = &hf->scheduler;

        uint32_t nextTargetCycles =
            scheduler->lastTargetCycles + scheduler->desiredPeriodCycles;

        // Realtime gyro/filtering/PID tasks get complete priority
        uint32_t nowCycles = systemGetCycleCounter();

        int32_t loopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        if (loopRemainingCycles < -scheduler->desiredPeriodCycles) {
            // A task has so grossly overrun that at entire gyro cycle has been
            // skipped This is most likely to occur when connected to the
            // configurator via USB as the serial task is non-deterministic Recover
            // as best we can, advancing scheduling by a whole number of cycles
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
            checkCoreTasks(hf, loopRemainingCycles, nowCycles, nextTargetCycles);
        }

        int32_t newLoopRemainingCyles =
            cmpTimeCycles(nextTargetCycles, systemGetCycleCounter());

        if (newLoopRemainingCyles > scheduler->guardMargin) {
            checkDynamicTasks(hf, newLoopRemainingCyles, nextTargetCycles);
        }
    }

#if defined(__cplusplus)
}
#endif
