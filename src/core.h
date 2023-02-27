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

#include "core/mixer.h"
#include "imu.h"
#include "scheduler.h"
#include "task/accelerometer.h"
#include "task/attitude.h"
#include "task/receiver.h"
#include "task/skyranger.h"
#include "task/visualizer.h"

class Core {

    public:

        typedef enum {

            ARMING_UNREADY,
            ARMING_READY,
            ARMING_ARMED,
            ARMING_FAILSAFE

        } armingStatus_e;

    private:

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t GYRO_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        static constexpr float MAX_ARMING_ANGLE_DEG = 25;

        VehicleState m_vstate;

        uint32_t m_imuInterruptCount;

        Scheduler m_scheduler;

        armingStatus_e m_armingStatus;

        Imu * m_imu;

        std::vector<PidController *> * m_pids;

        Mixer * m_mixer;

        void checkFailsafe(const uint32_t usec)
        {
            static bool hadSignal;

            const auto haveSignal = receiverTask.haveSignal(usec);

            if (haveSignal) {
                hadSignal = true;
            }

            if (hadSignal && !haveSignal) {
                m_armingStatus = ARMING_FAILSAFE;
            }
        }

        bool safeToArm(const uint32_t usec)
        {
            // Avoid arming if switch starts down
            static bool auxSwitchWasOff;

            auto auxSwitchValue = getAux1();

            if (!auxSwitchWasOff) {
                auxSwitchWasOff = auxSwitchValue > 900 && auxSwitchValue < 1200;
            }

            const auto maxArmingAngle = Imu::deg2rad(MAX_ARMING_ANGLE_DEG);

            const auto imuIsLevel =
                fabsf(m_vstate.phi) < maxArmingAngle &&
                fabsf(m_vstate.theta) < maxArmingAngle;

            const auto gyroDoneCalibrating = !m_imu->gyroIsCalibrating();

            const auto haveReceiverSignal = receiverTask.haveSignal(usec);

            return
                auxSwitchWasOff &&
                gyroDoneCalibrating &&
                imuIsLevel &&
                receiverTask.throttleIsDown() &&
                haveReceiverSignal;
        }

        float getAux1(void)
        {
            return receiverTask.getRawAux1();
        }

        void checkArmingSwitch(void)
        {
            static bool aux1WasSet;

            if (getAux1() > 1500) {
                if (!aux1WasSet) {
                    m_armingStatus = ARMING_ARMED;
                }
                aux1WasSet = true;
            }
            else {
                if (aux1WasSet) {
                    m_armingStatus = ARMING_READY;
                }
                aux1WasSet = false;
            }
        }

    public:

        Msp msp;

        ReceiverTask receiverTask;

        AttitudeTask attitudeTask = AttitudeTask(m_vstate);

        SkyrangerTask skyrangerTask = SkyrangerTask(m_vstate);

        AccelerometerTask accelerometerTask; 

        VisualizerTask visualizerTask =
            VisualizerTask(msp, m_vstate, receiverTask, skyrangerTask);

        Core(Imu * imu, std::vector<PidController *> & pids, Mixer & mixer)
        {
            m_imu = imu;
            m_pids = &pids;
            m_mixer = &mixer;
        }

        bool mspParse(const uint8_t byte)
        {
            return visualizerTask.parse(byte);
        }

        void begin(const uint32_t clockSpeed)
        {
            attitudeTask.begin(m_imu);

            visualizerTask.begin(&receiverTask);

            m_imu->begin(clockSpeed);
        }

        armingStatus_e getArmingStatus(void)
        {
            return m_armingStatus;
        }

        bool gotRebootRequest(void)
        {
            return visualizerTask.gotRebootRequest();
        }

        void handleImuInterrupt(const uint32_t cycleCounter)
        {
            m_imuInterruptCount++;

            m_imu->handleInterrupt(cycleCounter);
        }

        void updateAccelerometer(const int16_t rawAccel[3])
        {
            m_imu->updateAccelerometer(rawAccel);
        }

        void updateArmingStatus(const uint32_t usec)
        {
            checkFailsafe(usec);

            switch (m_armingStatus) {

                case ARMING_UNREADY:
                    if (safeToArm(usec)) {
                        m_armingStatus = ARMING_READY;
                    }
                    break;

                case ARMING_READY:
                    if (safeToArm(usec)) {
                        checkArmingSwitch();
                    }
                    else {
                        m_armingStatus = ARMING_UNREADY;
                    }
                    break;

                case ARMING_ARMED:
                    checkArmingSwitch();
                    break;

                default: // failsafe
                    break;
            }
        }

        void step(int16_t rawGyro[3], const uint32_t usec, float mixmotors[])
        {
            auto angvels = m_imu->gyroRawToFilteredDps(rawGyro);

            m_vstate.dphi   = angvels.x;
            m_vstate.dtheta = angvels.y;
            m_vstate.dpsi   = angvels.z;

            Demands demands = receiverTask.getDemands();

            auto motors = m_mixer->step(
                    demands, m_vstate, m_pids, receiverTask.throttleIsDown(), usec);

            for (auto i=0; i<m_mixer->getMotorCount(); i++) {

                mixmotors[i] = motors.values[i];
            }
        }

        bool isDynamicTaskReady(const uint32_t nowCycles)
        {
            return m_scheduler.isDynamicReady(nowCycles);
        }

        uint32_t coreTaskPreUpdate(int32_t & loopRemainingCycles)
        {
            return m_scheduler.corePreUpdate(loopRemainingCycles);
        }

        void postRunTask(
                Task & task,
                const uint32_t usecStart,
                const uint32_t usecEnd,
                const uint32_t nowCycles,
                const uint32_t anticipatedEndCycles)
        {
            task.update(usecStart, usecEnd-usecStart);
            m_scheduler.updateDynamic(nowCycles, anticipatedEndCycles);
        }

        uint32_t getAnticipatedEndCycles(Task & task, const uint32_t nowCycles)
        {
            return m_scheduler.getAnticipatedEndCycles(task, nowCycles);
        }

        bool isCoreTaskReady(const uint32_t nowCycles)
        {
            return m_scheduler.isCoreReady(nowCycles);
        }

        void updateScheduler(const uint32_t nowCycles, const uint32_t nextTargetCycles)
        {
            m_scheduler.corePostUpdate(nowCycles);

            // Bring the m_scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = m_imuInterruptCount + GYRO_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (m_imuInterruptCount >= _terminalGyroRateCount) {
                // Calculate number of clock cycles on average between gyro
                // interrupts
                uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
                m_scheduler.desiredPeriodCycles = sampleCycles / GYRO_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += GYRO_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and
            // remove skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            auto gyroSkew =
                m_imu->getGyroSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = m_imuInterruptCount + GYRO_LOCK_COUNT;
            }

            if (m_imuInterruptCount >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                m_scheduler.lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }
        }

        void prioritizeCoreTasks(Task::prioritizer_t & prioritizer, const uint32_t usec)
        {
            receiverTask.prioritize(usec, prioritizer);
            attitudeTask.prioritize(usec, prioritizer);
            visualizerTask.prioritize(usec, prioritizer);
        }

}; // class Core
