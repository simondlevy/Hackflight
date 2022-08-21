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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "arming.h"
#include "deg2rad.h"
#include "failsafe.h"
#include "gyro.h"
#include "hackflight_core.h"
#include "imu.h"
#include "led.h"
#include "motor.h"
#include "msp.h"
#include "receiver.h"
#include "scheduler.h"
#include "system.h"
#include "tasks/attitude.h"
#include "tasks/msp.h"
#include "tasks/receiver.h"

class Hackflight {

    private:

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t CORE_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        // Arming safety  
        static constexpr float MAX_ARMING_ANGLE = 25;

        Imu *                m_imu;
        Imu::align_fun       m_imuAlignFun;
        AnglePidController * m_anglePid;
        uint8_t              m_imuInterruptPin;
        uint8_t              m_ledPin;
        Mixer *              m_mixer;
        void *               m_motorDevice;
        Receiver *           m_receiver;
        Scheduler            m_scheduler;

    public:

        typedef struct {

            Task::data_t   taskData;

            AttitudeTask attitudeTask;
            MspTask      mspTask;
            ReceiverTask receiverTask;

        } data_t;

        void checkCoreTasks(data_t * data, uint32_t nowCycles)
        {
            Task::data_t * taskData = &data->taskData;

            int32_t loopRemainingCycles = m_scheduler.getLoopRemainingCycles();
            uint32_t nextTargetCycles = m_scheduler.getNextTargetCycles();

            m_scheduler.corePreUpdate();

            while (loopRemainingCycles > 0) {
                nowCycles = systemGetCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            taskData->gyro.readScaled(
                    taskData->imu,
                    m_imuAlignFun,
                    &taskData->vstate);

            uint32_t usec = timeMicros();

            float rawSetpoints[3] = {0,0,0};

            demands_t demands = {0,0,0,0};

            taskData->receiver->getDemands(usec, rawSetpoints, &demands);

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            HackflightCore::step(
                    &demands,
                    &taskData->vstate,
                    m_anglePid,
                    taskData->pidReset,
                    usec,
                    m_mixer,
                    mixmotors);

            for (uint8_t i=0; i<m_mixer->getMotorCount(); i++) {

                float motorOutput = mixmotors[i];

                motorOutput = motorValueLow() +
                    (motorValueHigh() - motorValueLow()) * motorOutput;

                if (taskData->failsafe.isActive()) {
                    if (motorIsProtocolDshot()) {
                        // Prevent getting into special reserved range
                        motorOutput = (motorOutput < motorValueLow()) ?
                            motorValueDisarmed() :
                            motorOutput; 
                    }
                    motorOutput = constrain_f(
                            motorOutput,
                            motorValueDisarmed(),
                            motorValueHigh());
                } else {
                    motorOutput =
                        constrain_f(
                                motorOutput,
                                motorValueLow(),
                                motorValueHigh());
                }
                mixmotors[i] = motorOutput;
            }

            motorWrite(taskData->motorDevice,
                    Arming::isArmed(&taskData->arming) ?
                    mixmotors :
                    taskData->mspMotors);

            m_scheduler.corePostUpdate(nowCycles);

            // Bring the scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = gyroDevInterruptCount() + CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (gyroDevInterruptCount() >= _terminalGyroRateCount) {
                // Calculate number of clock cycles on average between gyro
                // interrupts
                uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
                m_scheduler.desiredPeriodCycles = sampleCycles / CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += CORE_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and
            // remove skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            int32_t gyroSkew =
                Gyro::getSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = gyroDevInterruptCount() + GYRO_LOCK_COUNT;
            }

            if (gyroDevInterruptCount() >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                m_scheduler.lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }

        } // checkCoreTasks

        void checkDynamicTasks(data_t * full)
        {
            Task *selectedTask = NULL;
            uint16_t selectedTaskDynamicPriority = 0;

            uint32_t usec = timeMicros();

            Task::update(&full->receiverTask, &full->taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&full->attitudeTask, &full->taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&full->mspTask, &full->taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            if (selectedTask) {

                int32_t loopRemainingCycles =
                    m_scheduler.getLoopRemainingCycles();
                uint32_t nextTargetCycles =
                    m_scheduler.getNextTargetCycles();

                int32_t taskRequiredTimeUs = selectedTask->getRequiredTime();
                int32_t taskRequiredCycles =
                    (int32_t)systemClockMicrosToCycles(
                            (uint32_t)taskRequiredTimeUs);

                uint32_t nowCycles = systemGetCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);

                // Allow a little extra time
                taskRequiredCycles += m_scheduler.getTaskGuardCycles();

                if (taskRequiredCycles < loopRemainingCycles) {

                    uint32_t anticipatedEndCycles =
                        nowCycles + taskRequiredCycles;

                    selectedTask->execute(&full->taskData, usec);

                    m_scheduler.updateDynamic(
                            systemGetCycleCounter(),
                            anticipatedEndCycles);

                } else {
                    selectedTask->enableRun();
                }
            }

        } // checkDyanmicTasks

    public:

        Hackflight(
                Receiver * receiver,
                Imu * imu,
                Imu::align_fun imuAlignFun,
                AnglePidController * anglePid,
                Mixer * mixer,
                void * motorDevice,
                uint8_t imuInterruptPin,
                uint8_t ledPin)
        {
            m_receiver = receiver;
            m_imu = imu;
            m_mixer = mixer;
            m_imuAlignFun = imuAlignFun;
            m_anglePid = anglePid;
            m_motorDevice = motorDevice;
            m_imuInterruptPin = imuInterruptPin;
            m_ledPin = ledPin;
        }

        void begin(data_t * data)
        {
            Task::data_t * taskData = &data->taskData;

            taskData->receiver = m_receiver;
            taskData->imu = m_imu;
            taskData->motorDevice = m_motorDevice;

            // Initialize quaternion in upright position
            taskData->imuFusionPrev.quat.w = 1;

            taskData->maxArmingAngle = deg2rad(MAX_ARMING_ANGLE);

            data->taskData.msp.begin();

            imuDevInit(m_imuInterruptPin);

            ledDevInit(m_ledPin);

            Led::flash(10, 50);

            data->taskData.receiver->begin();
        }

        void step(data_t * data)
        {
            // Realtime gyro/filtering/PID tasks get complete priority
            uint32_t nowCycles = systemGetCycleCounter();

            if (m_scheduler.isCoreReady(nowCycles)) {
                checkCoreTasks(data, nowCycles);
            }

            if (m_scheduler.isDynamicReady(systemGetCycleCounter())) {
                checkDynamicTasks(data);
            }
        }

}; // class Hackflight
