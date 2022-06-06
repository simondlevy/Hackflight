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

#if defined(__cplusplus)
extern "C" {
#endif

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "gyro.h"
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
    static const uint32_t TASK_EXEC_TIME_SHIFT =   7;

    // Make aged tasks more schedulable
    static const uint32_t TASK_AGE_EXPEDITE_COUNT =   1;   

    // By scaling their expected execution time
    static const float    TASK_AGE_EXPEDITE_SCALE =   0.9; 

    // Gyro interrupt counts over which to measure loop time and skew
    static const uint32_t GYRO_RATE_COUNT = 25000;
    static const uint32_t GYRO_LOCK_COUNT = 400;

    // MSP task ---------------------------------------------------------------------

    static const uint32_t MSP_TASK_RATE = 100;

    static void task_msp(uint32_t time)
    {
        (void)time;

        mspUpdate(&_state, &_rx_axes, _armed, _mspmotors);
    }

    static task_t _mspTask;

    // Support for dynamically scheduled tasks ---------------------------------------

    static timeDelta_t taskNextStateTime;

    static void adjustDynamicPriority(task_t * task, timeUs_t currentTimeUs) 
    {
        // Task is time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
        // Task age is calculated from last execution
        task->taskAgeCycles =
            (cmpTimeUs(currentTimeUs, task->lastExecutedAtUs) / task->desiredPeriodUs);
        if (task->taskAgeCycles > 0) {
            task->dynamicPriority = 1 + task->taskAgeCycles;
        }
    }

    // Increase priority for RX task
    static void adjustRxDynamicPriority(task_t * task, timeUs_t currentTimeUs) 
    {
        if (task->dynamicPriority > 0) {
            task->taskAgeCycles = 1 + (cmpTimeUs(currentTimeUs,
                        task->lastSignaledAtUs) / task->desiredPeriodUs);
            task->dynamicPriority = 1 + task->taskAgeCycles;
        } else  {
            if (rxCheck(currentTimeUs)) {
                task->lastSignaledAtUs = currentTimeUs;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 2;
            } else {
                task->taskAgeCycles = 0;
            }
        }
    }

    static int32_t _schedLoopStartCycles;
    static int32_t _schedLoopStartMinCycles;
    static int32_t _schedLoopStartMaxCycles;
    static uint32_t _schedLoopStartDeltaDownCycles;
    static uint32_t _schedLoopStartDeltaUpCycles;

    static int32_t _taskGuardCycles;
    static int32_t _taskGuardMinCycles;
    static int32_t _taskGuardMaxCycles;
    static uint32_t _taskGuardDeltaDownCycles;
    static uint32_t _taskGuardDeltaUpCycles;

    static int32_t _desiredPeriodCycles;
    static uint32_t _lastTargetCycles;

    static uint32_t _nextTimingCycles;

    static void executeTask(task_t *task, timeUs_t currentTimeUs)
    {
        task->lastExecutedAtUs = currentTimeUs;
        task->dynamicPriority = 0;

        uint32_t time = timeMicros();
        task->fun(currentTimeUs);

        timeUs_t taskExecutionTimeUs = timeMicros() - time;

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
            int32_t schedLoopRemainingCycles,
            uint32_t nowCycles,
            uint32_t nextTargetCycles)
    {
        if (_schedLoopStartCycles > _schedLoopStartMinCycles) {
            _schedLoopStartCycles -= _schedLoopStartDeltaDownCycles;
        }

        while (schedLoopRemainingCycles > 0) {
            nowCycles = systemGetCycleCounter();
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

        hackflightRunCoreTasks(hf);

        // CPU busy
        if (cmpTimeCycles(_nextTimingCycles, nowCycles) < 0) {
            _nextTimingCycles += systemClockMicrosToCycles(1000000);
        }
        _lastTargetCycles = nextTargetCycles;

        // Bring the scheduler into lock with the gyro
        // Track the actual gyro rate over given number of cycle times and set the
        // expected timebase
        static uint32_t _terminalGyroRateCount;
        static int32_t _sampleRateStartCycles;

        if ((_terminalGyroRateCount == 0)) {
            _terminalGyroRateCount = gyroInterruptTime() + GYRO_RATE_COUNT;
            _sampleRateStartCycles = nowCycles;
        }

        if (gyroInterruptTime() >= _terminalGyroRateCount) {
            // Calculate the number of clock cycles on average between gyro interrupts
            uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
            _desiredPeriodCycles = sampleCycles / GYRO_RATE_COUNT;
            _sampleRateStartCycles = nowCycles;
            _terminalGyroRateCount += GYRO_RATE_COUNT;
        }

        // Track the actual gyro rate over given number of cycle times and remove skew
        static uint32_t _terminalGyroLockCount;
        static int32_t _gyroSkewAccum;

        int32_t gyroSkew = cmpTimeCycles(nextTargetCycles, gyroSyncTime()) %
            _desiredPeriodCycles;
        if (gyroSkew > (_desiredPeriodCycles / 2)) {
            gyroSkew -= _desiredPeriodCycles;
        }

        _gyroSkewAccum += gyroSkew;

        if ((_terminalGyroLockCount == 0)) {
            _terminalGyroLockCount = gyroInterruptTime() + GYRO_LOCK_COUNT;
        }

        if (gyroInterruptTime() >= _terminalGyroLockCount) {
            _terminalGyroLockCount += GYRO_LOCK_COUNT;

            // Move the desired start time of the gyroSampleTask
            _lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

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

    static void checkDynamicTasks(int32_t schedLoopRemainingCycles,
            uint32_t nextTargetCycles)
    {
        task_t *selectedTask = NULL;
        uint16_t selectedTaskDynamicPriority = 0;

        timeUs_t currentTimeUs = timeMicros();

        for (uint8_t k=0; k<_sensor_task_count; ++k) {
            task_t * task = &_sensor_tasks[k];
            adjustAndUpdateTask(task, currentTimeUs,&selectedTask,
                    &selectedTaskDynamicPriority);
        }

        adjustRxDynamicPriority(&_rxTask, currentTimeUs);
        updateDynamicTask(&_rxTask, &selectedTask, &selectedTaskDynamicPriority);

        adjustAndUpdateTask(&_attitudeTask, currentTimeUs,
                &selectedTask, &selectedTaskDynamicPriority);

        adjustAndUpdateTask(&_mspTask, currentTimeUs,
                &selectedTask, &selectedTaskDynamicPriority);

        if (selectedTask) {

            timeDelta_t taskRequiredTimeUs =
                selectedTask->anticipatedExecutionTime >> TASK_EXEC_TIME_SHIFT;
            int32_t taskRequiredTimeCycles =
                (int32_t)systemClockMicrosToCycles((uint32_t)taskRequiredTimeUs);

            uint32_t nowCycles = systemGetCycleCounter();
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

            // Allow a little extra time
            taskRequiredTimeCycles += _taskGuardCycles;

            if (taskRequiredTimeCycles < schedLoopRemainingCycles) {
                uint32_t antipatedEndCycles = nowCycles + taskRequiredTimeCycles;
                executeTask(selectedTask, currentTimeUs);
                nowCycles = systemGetCycleCounter();
                int32_t cyclesOverdue = cmpTimeCycles(nowCycles, antipatedEndCycles);

                if ((cyclesOverdue > 0) || (-cyclesOverdue < _taskGuardMinCycles)) {
                    if (_taskGuardCycles < _taskGuardMaxCycles) {
                        _taskGuardCycles += _taskGuardDeltaUpCycles;
                    }
                } else if (_taskGuardCycles > _taskGuardMinCycles) {
                    _taskGuardCycles -= _taskGuardDeltaDownCycles;
                }
            } else if (selectedTask->taskAgeCycles > TASK_AGE_EXPEDITE_COUNT) {
                // If a task has been unable to run, then reduce it's recorded
                // estimated run time to ensure it's ultimate scheduling
                selectedTask->anticipatedExecutionTime *= TASK_AGE_EXPEDITE_SCALE;
            }
        }
    }

    // ----------------------------------------------------------------------------

    void hackflightFullInit(
        hackflight_t * hackflight,
        void (*accel_fun)(uint32_t time),
         uint32_t accel_rate)
    {
        // Tuning constants for angle PID controller
        static const float RATE_P  = 1.441305;
        static const float RATE_I  = 19.55048;
        static const float RATE_D  = 0.021160;
        static const float RATE_F  = 0.0165048;
        static const float LEVEL_P = 0 /*3.0*/;

        void boardInit(void);
        boardInit();

        gyroInit();
        imuInit();
        ledInit();
        ledFlash(10, 50);
        failsafeInit();
        mspInit();
        armingSetDisabled(7);
        failsafeReset();

        hackflightInit(hackflight, RATE_P, RATE_I, RATE_D, RATE_F, LEVEL_P);

        // accel_fun can be traditional accelerometer, or hardware-fusion quaternion
        hackflightAddSensor(accel_fun, accel_rate);

        initTask(&_mspTask, task_msp, MSP_TASK_RATE);

        _schedLoopStartCycles = systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
        _schedLoopStartMinCycles = systemClockMicrosToCycles(SCHED_START_LOOP_MIN_US);
        _schedLoopStartMaxCycles = systemClockMicrosToCycles(SCHED_START_LOOP_MAX_US);
        _schedLoopStartDeltaDownCycles =
            systemClockMicrosToCycles(1) / SCHED_START_LOOP_DOWN_STEP;
        _schedLoopStartDeltaUpCycles =
            systemClockMicrosToCycles(1) / SCHED_START_LOOP_UP_STEP;

        _taskGuardMinCycles = systemClockMicrosToCycles(TASK_GUARD_MARGIN_MIN_US);
        _taskGuardMaxCycles = systemClockMicrosToCycles(TASK_GUARD_MARGIN_MAX_US);
        _taskGuardCycles = _taskGuardMinCycles;
        _taskGuardDeltaDownCycles =
            systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
        _taskGuardDeltaUpCycles =
            systemClockMicrosToCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

        _lastTargetCycles = systemGetCycleCounter();

        _nextTimingCycles = _lastTargetCycles;

        _desiredPeriodCycles = GYRO_PERIOD();
    }

    void hackflightStep(hackflight_t * hf)
    {
        // Realtime gyro/filtering/PID tasks get complete priority
        uint32_t nowCycles = systemGetCycleCounter();

        uint32_t nextTargetCycles = _lastTargetCycles + _desiredPeriodCycles;
        int32_t schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);

        if (schedLoopRemainingCycles < -_desiredPeriodCycles) {
            // A task has so grossly overrun that at entire gyro cycle has been
            // skipped This is most likely to occur when connected to the
            // configurator via USB as the serial task is non-deterministic Recover
            // as best we can, advancing scheduling by a whole number of cycles
            nextTargetCycles += _desiredPeriodCycles * (1 +
                    (schedLoopRemainingCycles / -_desiredPeriodCycles));
            schedLoopRemainingCycles = cmpTimeCycles(nextTargetCycles, nowCycles);
        }

        // Tune out the time lost between completing the last task execution and
        // re-entering the scheduler
        if ((schedLoopRemainingCycles < _schedLoopStartMinCycles) &&
                (_schedLoopStartCycles < _schedLoopStartMaxCycles)) {
            _schedLoopStartCycles += _schedLoopStartDeltaUpCycles;
        }

        // Once close to the timing boundary, poll for its arrival
        if (schedLoopRemainingCycles < _schedLoopStartCycles) {
            checkCoreTasks(hf, schedLoopRemainingCycles, nowCycles, nextTargetCycles);
        }

        schedLoopRemainingCycles =
            cmpTimeCycles(nextTargetCycles, systemGetCycleCounter());

        if ((schedLoopRemainingCycles >
                    (int32_t)systemClockMicrosToCycles(CHECK_GUARD_MARGIN_US))) {
            checkDynamicTasks(schedLoopRemainingCycles, nextTargetCycles);
        }
    }

#if defined(__cplusplus)
}
#endif
