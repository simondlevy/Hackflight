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
#include "imu.h"
#include "led.h"
#include "maths.h"
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

        // Initialzed in main()
        Receiver * m_receiver;
        Imu *      m_imu;
        Esc *      m_esc;
        Led *      m_led;
        Mixer *    m_mixer;
        
        vector<PidController *> * m_pidControllers;

        // Initialzed here
        Arming               m_arming;
        AttitudeTask         m_attitudeTask;
        bool                 m_failsafeIsActive;
        Imu::align_fun       m_imuAlignFun;
        MspTask              m_mspTask;
        ReceiverTask         m_rxTask;
        Receiver::sticks_t   m_rxSticks;
        Scheduler            m_scheduler;
        VehicleState         m_vstate;

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

            m_imu->readScaledGyro(m_imuAlignFun, &m_vstate);

            auto usec = timeMicros();

            float rawSetpoints[3] = {0,0,0};

            Demands demands = {0,0,0,0};

            m_receiver->getDemands(usec, rawSetpoints, &demands);

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            auto motors = m_mixer->step(
                    demands,
                    m_vstate,
                    m_pidControllers,
                    m_rxTask.gotPidReset(),
                    usec);

            for (auto i=0; i<m_mixer->getMotorCount(); i++) {

                auto motorOutput = motors.values[i];

                motorOutput = m_esc->valueLow() +
                    (m_esc->valueHigh() -
                     m_esc->valueLow()) * motorOutput;

                if (m_failsafeIsActive) {
                    if (m_esc->isProtocolDshot()) {
                        // Prevent getting into special reserved range
                        motorOutput = (motorOutput < m_esc->valueLow()) ?
                            m_esc->valueDisarmed() :
                            motorOutput; 
                    }
                    motorOutput = constrain_f(
                            motorOutput,
                            m_esc->valueDisarmed(),
                            m_esc->valueHigh());
                } else {
                    motorOutput =
                        constrain_f(
                                motorOutput,
                                m_esc->valueLow(),
                                m_esc->valueHigh());
                }
                mixmotors[i] = motorOutput;
            }

            m_esc->write(m_arming.isArmed() ?  mixmotors : m_mspTask.motors);

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

            Task::update(&m_rxTask, usec, &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&m_attitudeTask, usec,
                    &selectedTask, &selectedTaskDynamicPriority);

            Task::update(&m_mspTask, usec, &selectedTask, &selectedTaskDynamicPriority);

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

                    selectedTask->execute(usec);

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
            m_receiver = &receiver;
            m_imu = &imu;
            m_mixer = &mixer;
            m_imuAlignFun = imuAlignFun;
            m_esc = &esc;
            m_led = &led;

            m_pidControllers = &pidControllers;
        }

        void begin(void)
        {
            m_arming.begin(m_led);

            m_attitudeTask.begin(m_imu, &m_arming, &m_vstate);

            m_rxTask.begin(m_receiver, m_esc, &m_arming, &m_rxSticks);

            m_mspTask.begin(m_esc, &m_arming, &m_rxSticks, &m_vstate);

            m_imu->begin();

            m_esc->begin();

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
