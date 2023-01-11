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
#include "receiver.h"
#include "scheduler.h"
#include "task/accelerometer.h"
#include "task/attitude.h"
#include "task/visualizer.h"
#include "task/receiver.h"
#include "task/sensors.h"

class Board {

    private:

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t CORE_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        bool         m_failsafeIsActive;
        Led          m_led;
        Scheduler    m_scheduler;
        VehicleState m_vstate;

        Arming m_arming = Arming(m_led);

        SensorsTask m_sensorsTask = SensorsTask(m_vstate);

        AttitudeTask m_attitudeTask = AttitudeTask(m_arming, m_vstate);

        AccelerometerTask m_accelerometerTask;

        Msp m_msp;

        VisualizerTask m_visualizerTask =
            VisualizerTask(m_msp, m_arming, m_vstate, m_sensorsTask);

        ReceiverTask m_receiverTask   = ReceiverTask(m_arming);

        // Initialzed in sketch()
        Imu *   m_imu;
        Esc *   m_esc;
        Mixer * m_mixer;
        vector<PidController *> * m_pidControllers;

        void checkCoreTasks(uint32_t nowCycles)
        {
            int32_t loopRemainingCycles = m_scheduler.getLoopRemainingCycles();
            uint32_t nextTargetCycles = m_scheduler.getNextTargetCycles();

            m_scheduler.corePreUpdate();

            while (loopRemainingCycles > 0) {
                nowCycles = getCycleCounter();
                loopRemainingCycles =
                    intcmp(nextTargetCycles, nowCycles);
            }

            if (m_imu->gyroIsReady()) {

                auto angvels = m_imu->readGyroDps();

                m_vstate.dphi   = angvels.x;
                m_vstate.dtheta = angvels.y;
                m_vstate.dpsi   = angvels.z;
            }

            Demands demands = m_receiverTask.receiver->getDemands();

            delayMicroseconds(10);

            auto motors = m_mixer->step(
                    demands,
                    m_vstate,
                    m_pidControllers,
                    m_receiverTask.receiver->gotPidReset(),
                    micros());

            float mixmotors[MAX_SUPPORTED_MOTORS] = {0};

            for (auto i=0; i<m_mixer->getMotorCount(); i++) {

                mixmotors[i] = m_esc->getMotorValue(motors.values[i], m_failsafeIsActive);
            }

            m_esc->write(m_arming.isArmed() ?  mixmotors : m_visualizerTask.motors);

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

        void runTask(Task & task, uint32_t usec)
        {
            const auto nextTargetCycles = m_scheduler.getNextTargetCycles();

            const auto taskRequiredTimeUs = task.getRequiredTime();

            const auto nowCycles = getCycleCounter();

            const auto loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);

            // Allow a little extra time
            const auto taskRequiredCycles =
                (int32_t)microsecondsToClockCycles((uint32_t)taskRequiredTimeUs) +
                m_scheduler.getTaskGuardCycles();

            if (taskRequiredCycles < loopRemainingCycles) {

                const auto anticipatedEndCycles = nowCycles + taskRequiredCycles;

                task.execute(usec);

                m_scheduler.updateDynamic(getCycleCounter(), anticipatedEndCycles);
            } else {
                task.enableRun();
            }
        }

        void checkDynamicTasks(void)
        {
            const uint32_t usec = micros();

            Task::prioritizer_t prioritizer = {Task::NONE, 0};

            m_accelerometerTask.prioritize(usec, prioritizer);
            m_receiverTask.prioritize(usec, prioritizer);
            m_attitudeTask.prioritize(usec, prioritizer);
            m_visualizerTask.prioritize(usec, prioritizer);
            m_sensorsTask.prioritize(usec, prioritizer);

            if (m_visualizerTask.gotRebootRequest()) {
                reboot();
            }

            switch (prioritizer.id) {
                
                case Task::ACCELEROMETER:
                    runTask(m_accelerometerTask, usec);
                    break;

                case Task::ATTITUDE:
                    runTask(m_attitudeTask, usec);
                    break;

                case Task::VISUALIZER:
                    runTask(m_visualizerTask, usec);
                    break;

                case Task::RECEIVER:
                    runTask(m_receiverTask, usec);
                    break;

                case Task::SENSORS:
                    runTask(m_sensorsTask, usec);
                    break;
            
                case Task::NONE:
                    break;
             }
        }

        void parseSensors(const uint8_t byte)
        {
            m_sensorsTask.parse(byte);
        }

    protected:

        Board(
                Receiver & receiver,
                Imu & imu,
                vector<PidController *> & pidControllers,
                Mixer & mixer,
                Esc & esc,
                const int8_t ledPin)
        {
            m_receiverTask.receiver = &receiver;

            m_imu = &imu;
            m_pidControllers = &pidControllers;
            m_mixer = &mixer;
            m_esc = &esc;

            m_led.pin = ledPin < 0 ? -ledPin : ledPin;
            m_led.inverted = ledPin < 0;

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

            m_arming.begin(m_esc);

            m_attitudeTask.begin(m_imu);

            m_visualizerTask.begin(m_esc, m_receiverTask.receiver);

            m_imu->begin();

            m_esc->begin();

            m_led.begin();
            m_led.flash(10, 50);
        }

        void step(void)
        {
            // Realtime gyro/filtering/PID task get complete priority
            auto nowCycles = getCycleCounter();

            if (m_scheduler.isCoreReady(nowCycles)) {
                checkCoreTasks(nowCycles);
            }

            if (m_scheduler.isDynamicReady(getCycleCounter())) {
                checkDynamicTasks();
            }
        }

        void step(HardwareSerial & serial)
        {
            step();
            
            while (m_sensorsTask.imuDataAvailable()) {
                serial.write(m_sensorsTask.readImuData());
            }
        }

        void handleSerialEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                parseSensors(serial.read());
            }
        }

        static void setInterrupt(const uint8_t pin, void (*irq)(void), const uint32_t mode)
        {
            pinMode(pin, INPUT);
            attachInterrupt(pin, irq, mode);  
        }

}; // class Board
