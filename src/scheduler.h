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

// Constants -------------------------------------------------------

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

// Structure  ------------------------------------------------------------------

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


