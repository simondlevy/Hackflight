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
#include "scheduler.h"
#include "system.h"
#include "tasks/attitude.h"
#include "tasks/msp.h"

class Hackflight {

    private:

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t CORE_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        // Arming safety  
        static constexpr float MAX_ARMING_ANGLE = 25;

    public:

        typedef struct {

            HackflightCore::data_t coreData;

            imu_align_fun imuAlignFun;
            Task::data_t   taskData;
            Scheduler     scheduler;

            Failsafe      failsafe;

            AttitudeTask attitudeTask;
            MspTask      mspTask;

        } data_t;

    private:

        static void checkCoreTasks(
                data_t * data,
                Scheduler * scheduler,
                uint32_t nowCycles)
        {
            HackflightCore::data_t * coreData = &data->coreData;
            Task::data_t * taskData = &data->taskData;

            int32_t loopRemainingCycles = scheduler->getLoopRemainingCycles();
            uint32_t nextTargetCycles = scheduler->getNextTargetCycles();

            scheduler->corePreUpdate();

            while (loopRemainingCycles > 0) {
                nowCycles = systemGetCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            gyroReadScaled(
                    &taskData->gyro, data->imuAlignFun, &coreData->vstate);

            motorWrite(taskData->motorDevice,taskData->mspMotors);

            scheduler->corePostUpdate(nowCycles);

            // Bring the scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
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

            // Track actual gyro rate over given number of cycle times and
            // remove skew
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

        } // checkCoreTasks

        static void checkDynamicTasks(
                data_t * full, Scheduler * scheduler)
        {
            Task *selectedTask = NULL;
            uint16_t selectedTaskDynamicPriority = 0;

            uint32_t usec = timeMicros();

            Task::update(&full->attitudeTask, &full->taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&full->mspTask, &full->taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            if (selectedTask) {

                int32_t loopRemainingCycles =
                    scheduler->getLoopRemainingCycles();
                uint32_t nextTargetCycles =
                    scheduler->getNextTargetCycles();

                int32_t taskRequiredTimeUs = selectedTask->getRequiredTime();
                int32_t taskRequiredCycles =
                    (int32_t)systemClockMicrosToCycles(
                            (uint32_t)taskRequiredTimeUs);

                uint32_t nowCycles = systemGetCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);

                // Allow a little extra time
                taskRequiredCycles += scheduler->getTaskGuardCycles();

                if (taskRequiredCycles < loopRemainingCycles) {

                    uint32_t anticipatedEndCycles =
                        nowCycles + taskRequiredCycles;

                    selectedTask->execute
                        (&full->coreData, &full->taskData, usec);

                    scheduler->updateDynamic(
                            systemGetCycleCounter(),
                            anticipatedEndCycles);

                } else {
                    selectedTask->enableRun();
                }
            }

        } // checkDyanmicTasks

    public:

        static void init(
                data_t * full,
                void * motorDevice,
                uint8_t imuInterruptPin,
                imu_align_fun imuAlign,
                uint8_t ledPin)
        {
            //HackflightCore::data_t * coreData = &full->coreData;
            Task::data_t * taskData = &full->taskData;

            mspInit();
            gyroInit(&taskData->gyro);
            imuInit(imuInterruptPin);
            ledInit(ledPin);
            ledFlash(10, 50);
            failsafeInit();
            failsafeReset();

            full->imuAlignFun = imuAlign;

            taskData->motorDevice = motorDevice;

            // Initialize quaternion in upright position
            taskData->imuFusionPrev.quat.w = 1;

            taskData->maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);

        } // init

        static void step(data_t * full)
        {
            Scheduler * scheduler = &full->scheduler;

            // Realtime gyro/filtering/PID tasks get complete priority
            uint32_t nowCycles = systemGetCycleCounter();

            if (scheduler->isCoreReady(nowCycles)) {
                checkCoreTasks(full, scheduler, nowCycles);
            }

            if (scheduler->isDynamicReady(systemGetCycleCounter())) {
                checkDynamicTasks(full, scheduler);
            }
        }

}; // class Hackflight
