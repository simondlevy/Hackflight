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
#include "esc.h"
#include "imu.h"
#include "scheduler.h"
#include "task/accelerometer.h"
#include "task/attitude.h"
#include "task/receiver.h"
#include "task/skyranger.h"
#include "task/visualizer.h"

class Core {

    private:

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
                armingStatus = Core::ARMING_FAILSAFE;
            }
        }

        bool safeToArm(const uint32_t usec)
        {
            const auto maxArmingAngle = Imu::deg2rad(MAX_ARMING_ANGLE_DEG);

            const auto imuIsLevel =
                fabsf(vstate.phi) < maxArmingAngle &&
                fabsf(vstate.theta) < maxArmingAngle;

            const auto gyroDoneCalibrating = !imu->gyroIsCalibrating();

            const auto haveReceiverSignal = receiverTask.haveSignal(usec);

            return
                gyroDoneCalibrating &&
                imuIsLevel &&
                receiverTask.throttleIsDown() &&
                haveReceiverSignal;
        }

       void checkArmingSwitch(void)
        {
            static bool aux1WasSet;

            if (receiverTask.getRawAux1() > 1500) {
                if (!aux1WasSet) {
                    armingStatus = Core::ARMING_ARMED;
                }
                aux1WasSet = true;
            }
            else {
                if (aux1WasSet) {
                    armingStatus = Core::ARMING_READY;
                }
                aux1WasSet = false;
            }
        }

     public:

        typedef enum {

            ARMING_UNREADY,
            ARMING_READY,
            ARMING_ARMED,
            ARMING_FAILSAFE

        } armingStatus_e;

        armingStatus_e armingStatus;

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

        Esc * esc;

        std::vector<PidController *> * pidControllers;

        uint32_t imuInterruptCount;

        void updateArmingStatus(const uint32_t usec)
        {
            checkFailsafe(usec);

            switch (armingStatus) {

                case Core::ARMING_UNREADY:
                    if (safeToArm(usec)) {
                        armingStatus = Core::ARMING_READY;
                    }
                    break;

                case Core::ARMING_READY:
                    if (safeToArm(usec)) {
                        checkArmingSwitch();
                    }
                    else {
                        armingStatus = Core::ARMING_UNREADY;
                    }
                    break;

                case Core::ARMING_ARMED:
                    checkArmingSwitch();
                    break;

                default: // failsafe
                    break;
            }
        }


        void getMotorValues(int16_t rawGyro[3], const uint32_t usec, float mixmotors[])
        {
            imu->accumulateGyro();

            auto angvels = imu->gyroRawToFilteredDps(rawGyro);

            vstate.dphi   = angvels.x;
            vstate.dtheta = angvels.y;
            vstate.dpsi   = angvels.z;

            Demands demands = receiverTask.getDemands();

            auto motors = mixer->step(
                    demands, vstate, pidControllers, receiverTask.throttleIsDown(), usec);

            for (auto i=0; i<mixer->getMotorCount(); i++) {

                mixmotors[i] = esc->getMotorValue(motors.values[i]);
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
                _terminalGyroRateCount = imuInterruptCount + Imu::CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (imuInterruptCount >= _terminalGyroRateCount) {
                // Calculate number of clock cycles on average between gyro
                // interrupts
                uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
                m_scheduler.desiredPeriodCycles = sampleCycles / Imu::CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += Imu::CORE_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and
            // remove skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            auto gyroSkew =
                imu->getGyroSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = imuInterruptCount + Imu::GYRO_LOCK_COUNT;
            }

            if (imuInterruptCount >= _terminalGyroLockCount) {
                _terminalGyroLockCount += Imu::GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                m_scheduler.lastTargetCycles -= (_gyroSkewAccum/Imu::GYRO_LOCK_COUNT);

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
