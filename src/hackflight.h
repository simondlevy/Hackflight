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

#undef min
#undef max

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <vector>
using namespace std;

#include "arming.h"
#include "core/mixer.h"
#include "esc.h"
#include "failsafe.h"
#include "imu.h"
#include "led.h"
#include "maths.h"
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
        
        vector<PidController *> * m_pidControllers;

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

            auto usec = timeMicros();

            float rawSetpoints[3] = {0,0,0};

            Demands demands = {0,0,0,0};

            m_taskData.receiver->getDemands(usec, rawSetpoints, &demands);

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            auto motors = m_mixer->step(
                    demands,
                    m_taskData.vstate,
                    m_pidControllers,
                    m_taskData.pidReset,
                    usec);

            for (auto i=0; i<m_mixer->getMotorCount(); i++) {

                auto motorOutput = motors.values[i];

                motorOutput = m_taskData.esc->valueLow() +
                    (m_taskData.esc->valueHigh() -
                     m_taskData.esc->valueLow()) * motorOutput;

                if (m_taskData.failsafe.isActive()) {
                    if (m_taskData.esc->isProtocolDshot()) {
                        // Prevent getting into special reserved range
                        motorOutput = (motorOutput < m_taskData.esc->valueLow()) ?
                            m_taskData.esc->valueDisarmed() :
                            motorOutput; 
                    }
                    motorOutput = constrain_f(
                            motorOutput,
                            m_taskData.esc->valueDisarmed(),
                            m_taskData.esc->valueHigh());
                } else {
                    motorOutput =
                        constrain_f(
                                motorOutput,
                                m_taskData.esc->valueLow(),
                                m_taskData.esc->valueHigh());
                }
                mixmotors[i] = motorOutput;
            }

            m_taskData.esc->write(
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
                _terminalGyroRateCount = m_imu->gyroInterruptCount() + CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (m_imu->gyroInterruptCount() >= _terminalGyroRateCount) {
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

            auto gyroSkew =
                m_imu->getGyroSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = m_imu->gyroInterruptCount() + GYRO_LOCK_COUNT;
            }

            if (m_imu->gyroInterruptCount() >= _terminalGyroLockCount) {
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

                auto loopRemainingCycles = m_scheduler.getLoopRemainingCycles();
                auto nextTargetCycles = m_scheduler.getNextTargetCycles();

                auto taskRequiredTimeUs = selectedTask->getRequiredTime();
                auto taskRequiredCycles =
                    (int32_t)systemClockMicrosToCycles(
                            (uint32_t)taskRequiredTimeUs);

                auto nowCycles = systemGetCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);

                // Allow a little extra time
                taskRequiredCycles += m_scheduler.getTaskGuardCycles();

                if (taskRequiredCycles < loopRemainingCycles) {

                    auto anticipatedEndCycles = nowCycles + taskRequiredCycles;

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
                Receiver & receiver,
                Imu & imu,
                Imu::align_fun imuAlignFun,
                vector<PidController *> & pidControllers,
                Mixer & mixer,
                Esc & esc,
                Led & led)
        {
            m_mixer = &mixer;
            m_imuAlignFun = imuAlignFun;
            m_led = &led;

            m_pidControllers = &pidControllers;

            m_taskData.receiver = &receiver;
            m_taskData.imu = &imu;
            m_taskData.esc = &esc;

            m_taskData.maxArmingAngle = Math::deg2rad(MAX_ARMING_ANGLE);

            m_taskData.arming.m_led = &led;
        }

        void begin(void)
        {
            m_taskData.receiver->begin();
            m_taskData.msp.begin();
            m_taskData.imu->begin();
            m_taskData.esc->begin();

            m_led->begin();

            m_led->flash(10, 50);
        }

        void step(void)
        {
            // Realtime gyro/filtering/PID tasks get complete priority
            auto nowCycles = systemGetCycleCounter();

            if (m_scheduler.isCoreReady(nowCycles)) {
                checkCoreTasks(nowCycles);
            }

            if (m_scheduler.isDynamicReady(systemGetCycleCounter())) {
                checkDynamicTasks();
            }
        }

}; // class Hackflight
