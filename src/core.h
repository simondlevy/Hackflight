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
#include "imu/softquat.h"
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

        Msp m_msp;
        uint8_t m_mspPayloadSize;
        uint8_t m_mspPayloadIndex;

        ReceiverTask receiverTask;

        AttitudeTask attitudeTask = AttitudeTask(m_vstate);

        AccelerometerTask accelerometerTask; 

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

        class Prioritizer {

            public:

                virtual void prioritizeExtras(
                        Task::prioritizer_t & prioritizer,
                        const uint32_t usec,
                        AccelerometerTask accelerometerTask,
                        SkyrangerTask skyrangerTask)
                {
                    (void)prioritizer;
                    (void)usec;
                    (void)accelerometerTask;
                    (void)skyrangerTask;
                }
        };

        class ExtraPrioritizer : public Prioritizer {

            virtual void prioritizeExtras(
                    Task::prioritizer_t & prioritizer,
                    const uint32_t usec,
                    AccelerometerTask accelerometerTask,
                    SkyrangerTask skyrangerTask)
            {
                accelerometerTask.prioritize(usec, prioritizer);
                skyrangerTask.prioritize(usec, prioritizer);
            }
        };

        Prioritizer      m_ordinaryPrioritizer;
        ExtraPrioritizer m_extraPrioritizer;
        Prioritizer *    m_prioritizer;

    public:

        VisualizerTask visualizerTask =
            VisualizerTask(m_msp, m_vstate, receiverTask, skyrangerTask);

        SkyrangerTask skyrangerTask = SkyrangerTask(m_vstate);

        Core(SoftQuatImu * imu, std::vector<PidController *> & pids, Mixer & mixer)
            : Core((Imu *)imu, pids, mixer)
        {
            m_prioritizer = &m_extraPrioritizer;
        }

        Core(Imu * imu, std::vector<PidController *> & pids, Mixer & mixer)
        {
            m_imu = imu;
            m_pids = &pids;
            m_mixer = &mixer;

            m_prioritizer = &m_ordinaryPrioritizer;
        }

        void setSbusValues(uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            receiverTask.setValues(chanvals, usec, lostFrame, 172, 1811);
        }

        void setDsmxValues(uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            receiverTask.setValues(chanvals, usec, lostFrame, 988, 2011);
        }

        bool mspParse(const uint8_t byte)
        {
            bool result = visualizerTask.parse(byte);
            if (result) {
                m_mspPayloadSize = m_msp.payloadSize;
                m_mspPayloadIndex = 0;
            }
            return result;
        }

        uint8_t mspBytesAvailable(void)
        {
            return m_mspPayloadSize;
        }

        uint8_t mspGetByte(void)
        {
            const auto byte = m_msp.payload[m_mspPayloadIndex];
            m_mspPayloadIndex++;
            m_mspPayloadSize--;
            return byte;
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

        void runTask(const Task::id_t taskId, const uint32_t usec)
        {
            switch (taskId) {

                case Task::ATTITUDE:
                    attitudeTask.run(usec);
                    break;

                case Task::RECEIVER:
                    receiverTask.run(usec);
                    break;

                case Task::VISUALIZER:
                    visualizerTask.run(usec);
                    break;

                case Task::ACCELEROMETER:
                    accelerometerTask.run(usec);
                    break;

                case Task::SKYRANGER:
                    skyrangerTask.run(usec);
                    break;

                default:
                    break;
            }
        }

        void postRunTask(
                const Task::id_t taskId,
                const uint32_t usecStart,
                const uint32_t usecEnd,
                const uint32_t nowCycles,
                const uint32_t anticipatedEndCycles)
        {
            auto duration = usecEnd-usecStart;

            switch (taskId) {

                case Task::ATTITUDE:
                    attitudeTask.update(usecStart, duration);
                    break;

                case Task::RECEIVER:
                    receiverTask.update(usecStart, duration);
                    break;

                case Task::VISUALIZER:
                    visualizerTask.update(usecStart, duration);
                    break;

                case Task::ACCELEROMETER:
                    accelerometerTask.update(usecStart, duration);
                    break;

                case Task::SKYRANGER:
                    skyrangerTask.update(usecStart, duration);
                    break;

                default:
                    break;
            }

            m_scheduler.updateDynamic(nowCycles, anticipatedEndCycles);
        }

        uint32_t getAnticipatedEndCycles(const uint32_t nowCycles, const Task::id_t taskId)
        {
            uint32_t cycles = 0;

            switch (taskId) {

                case Task::ATTITUDE:
                    cycles = m_scheduler.getAnticipatedEndCycles(attitudeTask, nowCycles);
                    break;

                case Task::RECEIVER:
                    cycles = m_scheduler.getAnticipatedEndCycles(receiverTask, nowCycles);
                    break;

                case Task::VISUALIZER:
                    cycles = m_scheduler.getAnticipatedEndCycles(visualizerTask, nowCycles);
                    break;

                case Task::ACCELEROMETER:
                    cycles =
                        m_scheduler.getAnticipatedEndCycles(accelerometerTask, nowCycles);
                    break;

                case Task::SKYRANGER:
                    cycles = m_scheduler.getAnticipatedEndCycles(skyrangerTask, nowCycles);
                    break;

                default:
                    break;
            }

            return cycles;
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

        Task::id_t prioritizeTasks(const int16_t rawAccel[3], const uint32_t usec)
        {
            Task::prioritizer_t prioritizer = {Task::NONE, 0};

            receiverTask.prioritize(usec, prioritizer);
            attitudeTask.prioritize(usec, prioritizer);
            visualizerTask.prioritize(usec, prioritizer);

            m_prioritizer->prioritizeExtras(
                    prioritizer, usec, accelerometerTask, skyrangerTask);

            switch (prioritizer.id) {

                case Task::ATTITUDE:
                case Task::RECEIVER:
                    updateArmingStatus(usec);
                    break;

                case Task::ACCELEROMETER:
                    updateAccelerometer(rawAccel);
                    break;

                default:
                    break;
            }

            return prioritizer.id;
        }

}; // class Core
