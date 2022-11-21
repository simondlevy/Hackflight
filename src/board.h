/*
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

#include <stdint.h>

#include <vector>
using namespace std;

#include "arming.h"
#include "core/mixer.h"
#include "esc.h"
#include "imu.h"
#include "led.h"
#include "maths.h"
#include "scheduler.h"
#include "tasks/attitude.h"
#include "tasks/msp.h"
#include "tasks/receiver.h"

class Board {

    private:

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t CORE_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        // Initialzed in main()
        Imu *                     m_imu;
        Esc *                     m_esc;
        Mixer *                   m_mixer;
        vector<PidController *> * m_pidControllers;
        Receiver *                m_receiver;

        // Initialzed here
        Arming         m_arming;
        AttitudeTask   m_attitude;
        bool           m_failsafeIsActive;
        Led            m_led;
        Msp            m_msp;
        Scheduler      m_scheduler;
        VehicleState   m_vstate;

        void checkCoreTasks(uint32_t nowCycles)
        {
            int32_t loopRemainingCycles = m_scheduler.getLoopRemainingCycles();
            uint32_t nextTargetCycles = m_scheduler.getNextTargetCycles();

            m_scheduler.corePreUpdate();

            while (loopRemainingCycles > 0) {
                nowCycles = getCycleCounter();
                loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);
            }

            if (m_imu->gyroIsReady()) {

                auto angvels = m_imu->readGyroDps();

                m_vstate.dphi   = angvels.x;
                m_vstate.dtheta = angvels.y;
                m_vstate.dpsi   = angvels.z;
            }

            Demands demands = m_receiver->getDemands();

            delayMicroseconds(10);

            auto motors = m_mixer->step(
                    demands,
                    m_vstate,
                    m_pidControllers,
                    m_receiver->gotPidReset(),
                    micros());

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            for (auto i=0; i<m_mixer->getMotorCount(); i++) {

                mixmotors[i] = m_esc->getMotorValue(motors.values[i], m_failsafeIsActive);
            }

            m_esc->write(m_arming.isArmed() ?  mixmotors : m_msp.motors);

            m_scheduler.corePostUpdate(nowCycles);

            // Bring the scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = m_imu->getGyroInterruptCount() + CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (m_imu->getGyroInterruptCount() >= _terminalGyroRateCount) {
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
                _terminalGyroLockCount = m_imu->getGyroInterruptCount() + GYRO_LOCK_COUNT;
            }

            if (m_imu->getGyroInterruptCount() >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                m_scheduler.lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }

        } // checkCoreTasks

        void checkDynamicTasks(void)
        {
            Task *selectedTask = NULL;
            uint16_t selectedTaskDynamicPriority = 0;

            const uint32_t usec = micros();

            m_receiver->update(usec, &selectedTask, &selectedTaskDynamicPriority);

            m_attitude.update(usec, &selectedTask, &selectedTaskDynamicPriority);

            m_msp.update(usec, &selectedTask, &selectedTaskDynamicPriority);

            if (m_msp.gotRebootRequest()) {
                reboot();
            }

            if (selectedTask) {

                const auto nextTargetCycles = m_scheduler.getNextTargetCycles();

                const auto taskRequiredTimeUs = selectedTask->getRequiredTime();

                const auto nowCycles = getCycleCounter();

                const auto loopRemainingCycles =
                    cmpTimeCycles(nextTargetCycles, nowCycles);

                // Allow a little extra time
                const auto taskRequiredCycles =
                    (int32_t)microsecondsToClockCycles((uint32_t)taskRequiredTimeUs) +
                    m_scheduler.getTaskGuardCycles();

                if (taskRequiredCycles < loopRemainingCycles) {

                    const auto anticipatedEndCycles = nowCycles + taskRequiredCycles;

                    selectedTask->execute(usec);

                    m_scheduler.updateDynamic(getCycleCounter(), anticipatedEndCycles);
                } else {
                    selectedTask->enableRun();
                }
            }

        } // checkDyanmicTasks

     protected:

        Board(
                Receiver & receiver,
                Imu & imu,
                vector<PidController *> & pidControllers,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin,
                const bool ledInverted)
        {
            m_receiver = &receiver;
            m_imu = &imu;
            m_pidControllers = &pidControllers;
            m_mixer = &mixer;
            m_esc = &esc;

            m_led.pin = ledPin;
            m_led.inverted = ledInverted;

            imu.m_board = this;
            esc.m_board = this;
            receiver.m_board = this;
        }

     public:

        uint32_t microsToCycles(uint32_t micros)
        {
            return getClockSpeed() / 1000000 * micros;
        }

        virtual uint32_t getClockSpeed(void)  = 0;

        virtual uint32_t getCycleCounter(void) = 0;

        virtual void reboot(void) { }

        virtual void startCycleCounter(void) = 0;

        virtual void dmaInit(const vector<uint8_t> * motorPins, const uint32_t outputFreq)
        {
            (void)motorPins;
            (void)outputFreq;
        }

        virtual void dmaUpdateComplete(void)
        {
        }

        virtual void dmaUpdateStart(void)
        {
        }

        virtual void dmaWriteMotor(uint8_t index, uint16_t packet)
        {
            (void)index;
            (void)packet;
        }

        void begin(void)
        {
            startCycleCounter();

            m_arming.begin(m_esc, &m_led);

            m_attitude.begin(m_imu, &m_arming, &m_vstate);

            m_msp.begin(m_esc, &m_arming, m_receiver, &m_vstate);

            m_receiver->begin(&m_arming);

            m_imu->begin();

            m_esc->begin();

            m_led.begin();
            m_led.flash(10, 50);
        }

        void step(void)
        {
            // Realtime gyro/filtering/PID tasks get complete priority
            auto nowCycles = getCycleCounter();

            if (m_scheduler.isCoreReady(nowCycles)) {
                checkCoreTasks(nowCycles);
            }

            if (m_scheduler.isDynamicReady(getCycleCounter())) {
                checkDynamicTasks();
            }
        }

}; // class Board
