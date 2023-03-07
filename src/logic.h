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
#include "tasks/accelerometer.h"
#include "tasks/attitude.h"
#include "tasks/receiver.h"
#include "tasks/skyranger.h"
#include "tasks/visualizer.h"

class Logic {

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

        Scheduler m_scheduler;

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
                fabsf(vstate.phi) < maxArmingAngle &&
                fabsf(vstate.theta) < maxArmingAngle;

            const auto gyroDoneCalibrating = !imu->gyroIsCalibrating();

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

        armingStatus_e m_armingStatus;

     public:

        VehicleState vstate;

        Msp msp;

        ReceiverTask receiverTask;

        AttitudeTask attitudeTask = AttitudeTask(vstate);

        SkyrangerTask skyrangerTask = SkyrangerTask(vstate);

        AccelerometerTask accelerometerTask; 

        VisualizerTask visualizerTask =
            VisualizerTask(msp, vstate, receiverTask, skyrangerTask);

        Mixer * mixer;

        Imu * imu;

        std::vector<PidController *> * pidControllers;

        uint32_t imuInterruptCount;

        armingStatus_e getArmingStatus(void)
        {
            return m_armingStatus;
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
            auto angvels = imu->gyroRawToFilteredDps(rawGyro);

            vstate.dphi   = angvels.x;
            vstate.dtheta = angvels.y;
            vstate.dpsi   = angvels.z;

            Demands demands = receiverTask.getDemands();

            auto motors = mixer->step(
                    demands, vstate, pidControllers, receiverTask.throttleIsDown(), usec);

            for (auto i=0; i<mixer->getMotorCount(); i++) {

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
                _terminalGyroRateCount = imuInterruptCount + GYRO_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (imuInterruptCount >= _terminalGyroRateCount) {
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
                imu->getGyroSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = imuInterruptCount + GYRO_LOCK_COUNT;
            }

            if (imuInterruptCount >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                m_scheduler.lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }
        }

        void prioritizeTasks(Task::prioritizer_t & prioritizer, const uint32_t usec)
        {
            receiverTask.prioritize(usec, prioritizer);
            attitudeTask.prioritize(usec, prioritizer);
            visualizerTask.prioritize(usec, prioritizer);
        }

}; // class Logic
