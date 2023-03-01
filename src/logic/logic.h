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

#include "logic/core/mixer.h"
#include "logic/imu/softquat.h"
#include "logic/scheduler.h"
#include "logic/task/accelerometer.h"
#include "logic/task/attitude.h"
#include "logic/task/receiver.h"
#include "logic/task/skyranger.h"
#include "logic/task/visualizer.h"

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

        class Prioritizer {

            public:

                virtual void prioritizeExtras(
                        Task::prioritizer_t & prioritizer,
                        const uint32_t usec,
                        AccelerometerTask m_accelerometerTask,
                        SkyrangerTask m_skyrangerTask)
                {
                    (void)prioritizer;
                    (void)usec;
                    (void)m_accelerometerTask;
                    (void)m_skyrangerTask;
                }
        };

        class ExtraPrioritizer : public Prioritizer {

            virtual void prioritizeExtras(
                    Task::prioritizer_t & prioritizer,
                    const uint32_t usec,
                    AccelerometerTask m_accelerometerTask,
                    SkyrangerTask m_skyrangerTask)
            {
                m_accelerometerTask.prioritize(usec, prioritizer);
                m_skyrangerTask.prioritize(usec, prioritizer);
            }
        };

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

        ReceiverTask m_receiverTask;

        AttitudeTask m_attitudeTask = AttitudeTask(m_vstate);

        AccelerometerTask m_accelerometerTask; 

        SkyrangerTask m_skyrangerTask = SkyrangerTask(m_vstate);

        VisualizerTask m_visualizerTask =
            VisualizerTask(m_msp, m_vstate, m_receiverTask, m_skyrangerTask);

        Prioritizer      m_ordinaryPrioritizer;
        ExtraPrioritizer m_extraPrioritizer;
        Prioritizer *    m_prioritizer;

        Logic(
                Imu * imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                const uint32_t clockCyclesPerUsec)
        {
            m_imu = imu;
            m_pids = &pids;
            m_mixer = &mixer;

            m_prioritizer = &m_ordinaryPrioritizer;

            m_scheduler.init(clockCyclesPerUsec);
        }

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

        uint8_t skyrangerDataAvailable(void)
        {
            return m_skyrangerTask.imuDataAvailable();
        }

        uint8_t readSkyrangerData(void)
        {
            return m_skyrangerTask.readImuData();
        }

        void parseSkyrangerData(const uint8_t byte)
        {
            m_skyrangerTask.parse(byte);
        }

        void setSbusValues(uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            m_receiverTask.setValues(chanvals, usec, lostFrame, 172, 1811);
        }

        void setDsmxValues(uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            m_receiverTask.setValues(chanvals, usec, lostFrame, 988, 2011);
        }

        bool mspParse(const uint8_t byte)
        {
            bool result = m_visualizerTask.parse(byte);
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
            m_attitudeTask.begin(m_imu);

            m_visualizerTask.begin(&m_receiverTask);

            m_imu->begin(clockSpeed);
        }

        armingStatus_e getArmingStatus(void)
        {
            return m_armingStatus;
        }

        bool gotRebootRequest(void)
        {
            return m_visualizerTask.gotRebootRequest();
        }

        void handleImuInterrupt(const uint32_t cycleCounter)
        {
            m_imuInterruptCount++;

            m_imu->handleInterrupt(cycleCounter);
        }

        float * getMotors(int16_t rawGyro[3], const uint32_t usec)
        {
            auto angvels = m_imu->gyroRawToFilteredDps(rawGyro);

            m_vstate.dphi   = angvels.x;
            m_vstate.dtheta = angvels.y;
            m_vstate.dpsi   = angvels.z;

            Demands demands = m_receiverTask.getDemands();

            auto motors = m_mixer->step(
                    demands, m_vstate, m_pids, m_receiverTask.throttleIsDown(), usec);

            static float mixmotors[Motors::MAX_SUPPORTED];

            for (auto i=0; i<m_mixer->getMotorCount(); i++) {
                mixmotors[i] = motors.values[i];
            }

            return m_armingStatus == ARMING_ARMED ?
                mixmotors :
                m_visualizerTask.motors;
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
                    m_attitudeTask.run(usec);
                    break;

                case Task::RECEIVER:
                    m_receiverTask.run(usec);
                    break;

                case Task::VISUALIZER:
                    m_visualizerTask.run(usec);
                    break;

                case Task::ACCELEROMETER:
                    m_accelerometerTask.run(usec);
                    break;

                case Task::SKYRANGER:
                    m_skyrangerTask.run(usec);
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
                    m_attitudeTask.update(usecStart, duration);
                    break;

                case Task::RECEIVER:
                    m_receiverTask.update(usecStart, duration);
                    break;

                case Task::VISUALIZER:
                    m_visualizerTask.update(usecStart, duration);
                    break;

                case Task::ACCELEROMETER:
                    m_accelerometerTask.update(usecStart, duration);
                    break;

                case Task::SKYRANGER:
                    m_skyrangerTask.update(usecStart, duration);
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
                    cycles =
                        m_scheduler.getAnticipatedEndCycles(m_attitudeTask, nowCycles);
                    break;

                case Task::RECEIVER:
                    cycles =
                        m_scheduler.getAnticipatedEndCycles(m_receiverTask, nowCycles);
                    break;

                case Task::VISUALIZER:
                    cycles =
                        m_scheduler.getAnticipatedEndCycles(m_visualizerTask, nowCycles);
                    break;

                case Task::ACCELEROMETER:
                    cycles = 
                        m_scheduler.getAnticipatedEndCycles(
                                m_accelerometerTask, nowCycles);
                    break;

                case Task::SKYRANGER:
                    cycles =
                        m_scheduler.getAnticipatedEndCycles(m_skyrangerTask, nowCycles);
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

            m_receiverTask.prioritize(usec, prioritizer);
            m_attitudeTask.prioritize(usec, prioritizer);
            m_visualizerTask.prioritize(usec, prioritizer);

            m_prioritizer->prioritizeExtras(
                    prioritizer, usec, m_accelerometerTask, m_skyrangerTask);

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

    public:

        Logic(
                SoftQuatImu & imu,
                std::vector<PidController *> & pids,
                Mixer & mixer,
                const uint32_t clockCyclesPerUsec)
            : Logic((Imu *)&imu, pids, mixer, clockCyclesPerUsec)
        {
            m_prioritizer = &m_extraPrioritizer;
        }

        static uint8_t _skyrangerDataAvailable(void)
        {
            extern Logic g_logic;
            return g_logic.skyrangerDataAvailable();
        }

        static uint8_t _readSkyrangerData(void)
        {
            extern Logic g_logic;
            return g_logic.readSkyrangerData();
        }

        static void _parseSkyrangerData(const uint8_t byte)
        {
            extern Logic g_logic;
            g_logic.parseSkyrangerData(byte);
        }

        static void _setSbusValues(
                uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            extern Logic g_logic;
            g_logic.setSbusValues(chanvals, usec, lostFrame);
        }

        static void _setDsmxValues(
                uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            extern Logic g_logic;
            g_logic.setDsmxValues(chanvals, usec, lostFrame);
        }

        static bool _mspParse(const uint8_t byte)
        {
            extern Logic g_logic;
            return g_logic.mspParse(byte);
        }

        static uint8_t _mspBytesAvailable(void)
        {
            extern Logic g_logic;
            return g_logic.mspBytesAvailable();
        }

        static uint8_t _mspGetByte(void)
        {
            extern Logic g_logic;
            return g_logic.mspGetByte();
        }

        static void _begin(const uint32_t clockSpeed)
        {
            extern Logic g_logic;
            g_logic.begin(clockSpeed);
        }

        static armingStatus_e _getArmingStatus(void)
        {
            extern Logic g_logic;
            return g_logic.getArmingStatus();
        }

        static bool _gotRebootRequest(void)
        {
            extern Logic g_logic;
            return g_logic.gotRebootRequest();
        }

        static void _handleImuInterrupt(const uint32_t cycleCounter)
        {
            extern Logic g_logic;
            g_logic.handleImuInterrupt(cycleCounter);
        }

        static float * _getMotors(int16_t rawGyro[3], const uint32_t usec)
        {
            extern Logic g_logic;
            return g_logic.getMotors(rawGyro, usec);
        }

        static bool _isDynamicTaskReady(const uint32_t nowCycles)
        {
            extern Logic g_logic;
            return g_logic.isDynamicTaskReady(nowCycles);
        }

        static uint32_t _coreTaskPreUpdate(int32_t & loopRemainingCycles)
        {
            extern Logic g_logic;
            return g_logic.coreTaskPreUpdate(loopRemainingCycles);
        }

        static void _runTask(const Task::id_t taskId, const uint32_t usec)
        {
            extern Logic g_logic;
            g_logic.runTask(taskId, usec);
        }

        static void _postRunTask(
                const Task::id_t taskId,
                const uint32_t usecStart,
                const uint32_t usecEnd,
                const uint32_t nowCycles,
                const uint32_t anticipatedEndCycles)
        {
            extern Logic g_logic;
            g_logic.postRunTask(
                    taskId, usecStart, usecEnd, nowCycles, anticipatedEndCycles);
        }

        static uint32_t _getAnticipatedEndCycles(
                const uint32_t nowCycles, const Task::id_t taskId)
        {
            extern Logic g_logic;
            return g_logic.getAnticipatedEndCycles(nowCycles, taskId);
        }

        static bool _isCoreTaskReady(const uint32_t nowCycles)
        {
            extern Logic g_logic;
            return g_logic.isCoreTaskReady(nowCycles);
        }

        static void _updateScheduler(
                const uint32_t nowCycles, const uint32_t nextTargetCycles)
        {
            extern Logic g_logic;
            g_logic.updateScheduler(nowCycles, nextTargetCycles);
        }

        static Task::id_t _prioritizeTasks(
                const int16_t rawAccel[3], const uint32_t usec)
        {
            extern Logic g_logic;
            return g_logic.prioritizeTasks(rawAccel, usec);
        }

}; // class Logic
