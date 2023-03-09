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

        armingStatus_e m_armingStatus;

        VehicleState m_vstate;

        Msp m_msp;

        uint32_t m_imuInterruptCount;

        ReceiverTask m_receiverTask;

        AttitudeTask m_attitudeTask = AttitudeTask(m_vstate);

        VisualizerTask m_visualizerTask; 

        SkyrangerTask m_skyrangerTask = SkyrangerTask(m_vstate);

        AccelerometerTask m_acclerometerTask; 

        void checkFailsafe(const uint32_t usec)
        {
            static bool hadSignal;

            const auto haveSignal = m_receiverTask.haveSignal(usec);

            if (haveSignal) {
                hadSignal = true;
            }

            if (hadSignal && !haveSignal) {
                m_armingStatus = ARMING_FAILSAFE;
            }
        }

        bool safeToArm(Imu & imu, const uint32_t usec)
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

            const auto gyroDoneCalibrating = !imu.gyroIsCalibrating();

            const auto haveReceiverSignal = m_receiverTask.haveSignal(usec);

            return
                auxSwitchWasOff &&
                gyroDoneCalibrating &&
                imuIsLevel &&
                m_receiverTask.throttleIsDown() &&
                haveReceiverSignal;
        }

        float getAux1(void)
        {
            return m_receiverTask.getRawAux1();
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

        void begin(Imu & imu, const uint32_t clockSpeed)
        {
            m_attitudeTask.begin(imu);

            imu.begin(clockSpeed);
        }

        armingStatus_e getArmingStatus(void)
        {
            return m_armingStatus;
        }

        void updateAccelerometer(Imu & imu, const int16_t rawAccel[3])
        {
            imu.updateAccelerometer(rawAccel);
        }

        void handleImuInterrupt(Imu & imu, const uint32_t cycleCounter)
        {
            m_imuInterruptCount++;
            imu.handleInterrupt(cycleCounter);
        }

        void updateArmingStatus(Imu & imu, const uint32_t usec)
        {
            checkFailsafe(usec);

            switch (m_armingStatus) {

                case ARMING_UNREADY:
                    if (safeToArm(imu, usec)) {
                        m_armingStatus = ARMING_READY;
                    }
                    break;

                case ARMING_READY:
                    if (safeToArm(imu, usec)) {
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

        void step(
                Imu & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                int16_t rawGyro[3],
                const uint32_t usec,
                float motors[])
        {
            imu.gyroRawToFilteredDps(rawGyro, m_vstate);

            Demands demands = m_receiverTask.modifyDemands();

            auto pidReset = m_receiverTask.throttleIsDown();

            PidController::run(pids, demands, m_vstate, usec, pidReset);

            mixer.getMotors(demands, motors);
        }

        bool isDynamicTaskReady(const uint32_t nowCycles)
        {
            return m_scheduler.isDynamicReady(nowCycles);
        }

        uint32_t coreTaskPreUpdate(int32_t & loopRemainingCycles)
        {
            return m_scheduler.corePreUpdate(loopRemainingCycles);
        }

        void runTask(const Task::id_e id, const uint32_t usec)
        {
            switch (id) {

                case Task::ATTITUDE:
                    m_attitudeTask.run(usec);
                    break;

                case Task::VISUALIZER:
                    break;

                case Task::RECEIVER:
                    m_receiverTask.run(usec);
                    break;

                case Task::ACCELEROMETER:
                    m_acclerometerTask.run(usec);
                    break;

                case Task::SKYRANGER:
                    m_skyrangerTask.run(usec);
                    break;

                default:
                    break;
            }
        }

        void postRunTask(
                Task::id_e id,
                const uint32_t usecStart,
                const uint32_t usecEnd,
                const uint32_t nowCycles,
                const uint32_t anticipatedEndCycles)
        {
            const auto usecTaken = usecEnd-usecStart;

            switch (id) {

                case Task::ATTITUDE:
                    m_attitudeTask.update(usecStart, usecTaken);
                    break;

                case Task::VISUALIZER:
                    m_visualizerTask.update(usecStart, usecTaken);
                    break;

                case Task::RECEIVER:
                    m_receiverTask.update(usecStart, usecTaken);
                    break;

                case Task::ACCELEROMETER:
                    m_acclerometerTask.update(usecStart, usecTaken);
                    break;

                case Task::SKYRANGER:
                    m_skyrangerTask.update(usecStart, usecTaken);
                    break;

                default:
                    break;
            }

            m_scheduler.updateDynamic(nowCycles, anticipatedEndCycles);
        }

        uint32_t getTaskAnticipatedEndCycles(Task::id_e id, const uint32_t nowCycles)
        {
            uint32_t endCycles = 0;

            switch (id) {

                case Task::ATTITUDE:
                    endCycles = m_scheduler.getAnticipatedEndCycles(
                            m_attitudeTask, nowCycles);
                    break;

                case Task::VISUALIZER:
                    endCycles = m_scheduler.getAnticipatedEndCycles(
                            m_visualizerTask, nowCycles);
                    break;

                case Task::RECEIVER:
                    endCycles = m_scheduler.getAnticipatedEndCycles(
                            m_receiverTask, nowCycles);
                    break;

                case Task::ACCELEROMETER:
                    endCycles = m_scheduler.getAnticipatedEndCycles(
                            m_acclerometerTask, nowCycles);
                    break;

                case Task::SKYRANGER:
                    endCycles = m_scheduler.getAnticipatedEndCycles(
                            m_skyrangerTask, nowCycles);
                    break;

                default:
                    break;
            }

            return endCycles;
        }

        bool isCoreTaskReady(const uint32_t nowCycles)
        {
            return m_scheduler.isCoreReady(nowCycles);
        }

        void updateScheduler(
                Imu & imu,
                const uint32_t nowCycles,
                const uint32_t nextTargetCycles)
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
                imu.getGyroSkew(nextTargetCycles, m_scheduler.desiredPeriodCycles);

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

        void prioritizeTasks(Task::prioritizer_t & prioritizer, const uint32_t usec)
        {
            m_receiverTask.prioritize(usec, prioritizer);
            m_attitudeTask.prioritize(usec, prioritizer);
            m_visualizerTask.prioritize(usec, prioritizer);
        }

        uint8_t mspAvailable(void)
        {
            return m_msp.available();
        }

        uint8_t mspRead(void)
        {
            return m_msp.read();
        }

        bool mspParse(const uint8_t byte)
        {
            return m_visualizerTask.parse(
                    m_vstate, m_receiverTask, m_skyrangerTask, m_msp, byte);
        }

        uint8_t skyrangerDataAvailable(void)
        {
            return m_skyrangerTask.imuDataAvailable();
        }

        uint8_t skyrangerReadData(void)
        {
            return m_skyrangerTask.readImuData();
        }

        void skyrangerParseData(const uint8_t byte)
        {
            m_skyrangerTask.parse(byte);
        }

        float * getVisualizerMotors(void)
        {
            return m_visualizerTask.motors;
        }

        bool gotRebootRequest(void)
        {
            return m_visualizerTask.gotRebootRequest();
        }

        void setSbusValues(
                uint16_t chanvals[],
                const uint32_t usec,
                const bool lostFrame)
        {
            m_receiverTask.setValues(chanvals, usec, lostFrame, 172, 1811);
        }

        void setDsmxValues(
                uint16_t chanvals[],
                const uint32_t usec,
                const bool lostFrame)
        {
            m_receiverTask.setValues(chanvals, usec, lostFrame, 988, 2011);
        }

        void prioritizeExtraTasks(
                Task::prioritizer_t & prioritizer, const uint32_t usec)
        {
            m_acclerometerTask.prioritize(usec, prioritizer);
            m_skyrangerTask.prioritize(usec, prioritizer);
        }

}; // class Logic
