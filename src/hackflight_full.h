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
#include "failsafe.h"
#include "hackflight_core.h"
#include "imu.h"
#include "led.h"
#include "maths.h"
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

        // Initialzed in main()
        Imu   * m_imu;
        Led   * m_led;
        Mixer * m_mixer;
        
        // First PID controller is automatically AnglePidController;
        // others can be added via addPidController()
        PidController * m_pidControllers[PidController::MAX_PID_CONTROLLERS];
        uint8_t         m_pidControllerCount;

        // Initialzed here
        AttitudeTask         m_attitudeTask;
        Imu::align_fun       m_imuAlignFun;
        MspTask              m_mspTask;
        ReceiverTask         m_receiverTask;
        Scheduler            m_scheduler;
        Task::data_t         m_taskData;

        void checkCoreTasks(uint32_t nowCycles)
        {
            int32_t loopRemainingCycles = m_scheduler.getLoopRemainingCycles();
            uint32_t nextTargetCycles = m_scheduler.getNextTargetCycles();

            m_scheduler.corePreUpdate();

            while (loopRemainingCycles > 0) {
                nowCycles = systemGetCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            m_taskData.imu->readScaledGyro(
                    m_taskData.imu,
                    m_imuAlignFun,
                    &m_taskData.vstate);

            uint32_t usec = timeMicros();

            float rawSetpoints[3] = {0,0,0};

            demands_t demands = {0,0,0,0};

            m_taskData.receiver->getDemands(usec, rawSetpoints, &demands);

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            HackflightCore::step(
                    &demands,
                    &m_taskData.vstate,
                    m_pidControllers,
                    m_pidControllerCount,
                    m_taskData.pidReset,
                    usec,
                    m_mixer,
                    mixmotors);

            for (uint8_t i=0; i<m_mixer->getMotorCount(); i++) {

                float motorOutput = mixmotors[i];

                motorOutput = motorValueLow() +
                    (motorValueHigh() - motorValueLow()) * motorOutput;

                if (m_taskData.failsafe.isActive()) {
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

            motorWrite(m_taskData.motorDevice,
                    m_taskData.arming.isArmed() ?
                    mixmotors :
                    m_taskData.mspMotors);

            m_scheduler.corePostUpdate(nowCycles);

            // Bring the scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = imuDevGyroInterruptCount() + CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (imuDevGyroInterruptCount() >= _terminalGyroRateCount) {
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
                Imu::getGyroSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = imuDevGyroInterruptCount() + GYRO_LOCK_COUNT;
            }

            if (imuDevGyroInterruptCount() >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                m_scheduler.lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }

        }

        void checkDynamicTasks(void)
        {
            Task *selectedTask = NULL;
            uint16_t selectedTaskDynamicPriority = 0;

            uint32_t usec = timeMicros();

            Task::update(&m_receiverTask, &m_taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&m_attitudeTask, &m_taskData, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&m_mspTask, &m_taskData, usec,
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

                    selectedTask->execute(&m_taskData, usec);

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
                Led * led)
        {
            m_mixer = mixer;
            m_imuAlignFun = imuAlignFun;
            m_led = led;

            m_pidControllers[0] = anglePid;
            m_pidControllerCount = 1;

            m_taskData.receiver = receiver;
            m_taskData.imu = imu;
            m_taskData.motorDevice = motorDevice;

            // Initialize quaternion in upright position
            m_taskData.imuFusionPrev.quat.w = 1;

            m_taskData.maxArmingAngle = Math::deg2rad(MAX_ARMING_ANGLE);

            m_taskData.arming.m_led = led;
        }

        void begin(void)
        {
            m_taskData.receiver->begin();

            m_taskData.msp.begin();

            m_taskData.imu->begin();

            m_led->begin();

            m_led->flash(10, 50);
        }

        void step(void)
        {
            // Realtime gyro/filtering/PID tasks get complete priority
            uint32_t nowCycles = systemGetCycleCounter();

            if (m_scheduler.isCoreReady(nowCycles)) {
                checkCoreTasks(nowCycles);
            }

            if (m_scheduler.isDynamicReady(systemGetCycleCounter())) {
                checkDynamicTasks();
            }
        }

}; // class Hackflight
