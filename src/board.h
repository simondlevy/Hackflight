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
#include <stdarg.h>

#include <vector>
using namespace std;

#include "core/mixer.h"
#include "esc.h"
#include "imu.h"
#include "receiver.h"
#include "task/accelerometer.h"
#include "task/attitude.h"
#include "task/visualizer.h"
#include "task/receiver.h"
#include "task/sensors.h"

class Board {

    private:

        static constexpr float MAX_ARMING_ANGLE = 25;

        // Scheduling constants ------------------------------------------------

        // Wait at start of scheduler loop if gyroSampleTask is nearly due
        static const uint32_t SCHED_START_LOOP_MIN_US = 1;   
        static const uint32_t SCHED_START_LOOP_MAX_US = 12;

        // Fraction of a us to reduce start loop wait
        static const uint32_t SCHED_START_LOOP_DOWN_STEP = 50;  

        // Fraction of a us to increase start loop wait
        static const uint32_t SCHED_START_LOOP_UP_STEP = 1;   

        // Add an amount to the estimate of a task duration
        static const uint32_t TASK_GUARD_MARGIN_MIN_US = 3;   
        static const uint32_t TASK_GUARD_MARGIN_MAX_US = 6;

        // Fraction of a us to reduce task guard margin
        static const uint32_t TASK_GUARD_MARGIN_DOWN_STEP = 50;  

        // Fraction of a us to increase task guard margin
        static const uint32_t TASK_GUARD_MARGIN_UP_STEP = 1;   

        // Add a margin to the amount of time allowed for a check function to run
        static const uint32_t CHECK_GUARD_MARGIN_US = 2 ;  

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t CORE_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        // Scheduler variables
        uint32_t m_clockRate;
        int32_t  m_guardMargin;
        int32_t  m_loopRemainingCycles;
        int32_t  m_loopStartCycles;
        uint32_t m_loopStartDeltaDownCycles;
        uint32_t m_loopStartDeltaUpCycles;
        int32_t  m_loopStartMaxCycles;
        int32_t  m_loopStartMinCycles;
        uint32_t m_nextTargetCycles;
        uint32_t m_nextTimingCycles;
        int32_t  m_taskGuardCycles;
        uint32_t m_taskGuardDeltaDownCycles;
        uint32_t m_taskGuardDeltaUpCycles;
        int32_t  m_taskGuardMinCycles;
        int32_t  m_taskGuardMaxCycles;
        int32_t  desiredPeriodCycles;
        uint32_t lastTargetCycles;

        bool         m_failsafeIsActive;
        VehicleState m_vstate;

        // Arming guards
        bool m_accDoneCalibrating;
        bool m_angleOkay;
        bool m_gotFailsafe;
        bool m_gyroDoneCalibrating;
        bool m_haveSignal;
        bool m_isArmed;
        bool m_switchOkay;
        bool m_throttleIsDown;

        uint8_t m_ledPin;
        bool m_ledInverted;

        float m_maxArmingAngle = Imu::deg2rad(MAX_ARMING_ANGLE);

        SensorsTask m_sensorsTask = SensorsTask(m_vstate);

        AttitudeTask m_attitudeTask = AttitudeTask(m_vstate);

        AccelerometerTask m_accelerometerTask;

        Msp m_msp;

        VisualizerTask m_visualizerTask =
            VisualizerTask(m_msp, m_vstate, m_sensorsTask);

        ReceiverTask m_receiverTask;

        // Initialzed in sketch()
        Imu *   m_imu;
        Esc *   m_esc;
        Mixer * m_mixer;
        vector<PidController *> * m_pidControllers;

        void checkCoreTasks(uint32_t nowCycles)
        {
            int32_t loopRemainingCycles = getLoopRemainingCycles();
            uint32_t nextTargetCycles = getNextTargetCycles();

            corePreUpdate();

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

            m_esc->write(isArmed() ?  mixmotors : m_visualizerTask.motors);

            corePostUpdate(nowCycles);

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
                desiredPeriodCycles = sampleCycles / CORE_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += CORE_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and
            // remove skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            auto gyroSkew =
                m_imu->getGyroSkew(nextTargetCycles, desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = m_imu->getGyroInterruptCount() + GYRO_LOCK_COUNT;
            }

            if (m_imu->getGyroInterruptCount() >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }

        } // checkCoreTasks

        void runTask(Task & task, uint32_t usec)
        {
            const auto nextTargetCycles = getNextTargetCycles();

            const auto taskRequiredTimeUs = task.getRequiredTime();

            const auto nowCycles = getCycleCounter();

            const auto loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);

            // Allow a little extra time
            const auto taskRequiredCycles =
                (int32_t)microsecondsToClockCycles((uint32_t)taskRequiredTimeUs) +
                getTaskGuardCycles();

            if (taskRequiredCycles < loopRemainingCycles) {

                const auto anticipatedEndCycles = nowCycles + taskRequiredCycles;

                task.execute(usec);

                updateDynamic(getCycleCounter(), anticipatedEndCycles);
            } else {
                task.enableRun();
            }
        }

        void updateArmingFromReceiver(void)
        {
            Receiver * receiver = m_receiverTask.receiver;

            switch (receiver->getState()) {

                case Receiver::STATE_UPDATE:
                    attemptToArm(micros(), receiver->aux1IsSet());
                    break;

                case Receiver::STATE_CHECK:
                    updateFromReceiver(
                            receiver->throttleIsDown(),
                            receiver->aux1IsSet(),
                            receiver->hasSignal());
                    break;

                default:
                    break;
            }
        }

        void updateArmingFromImu(void)
        {
            const auto imuIsLevel =
                fabsf(m_vstate.phi) < m_maxArmingAngle &&
                fabsf(m_vstate.theta) < m_maxArmingAngle;

            updateArmingFromImu(imuIsLevel, m_imu->gyroIsCalibrating()); 
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
                    updateArmingFromImu();
                    break;

                case Task::VISUALIZER:
                    runTask(m_visualizerTask, usec);
                    break;

                case Task::RECEIVER:
                    runTask(m_receiverTask, usec);
                    updateArmingFromReceiver();
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

        static void outbuf(char * buf)
        {
            Serial.print(buf);
            Serial.flush();
        }

        bool readyToArm(void)
        {
            return 
                m_accDoneCalibrating &&
                m_angleOkay &&
                !m_gotFailsafe &&
                m_haveSignal &&
                m_gyroDoneCalibrating &&
                m_switchOkay &&
                m_throttleIsDown;
        }

        void disarm(void)
        {
            if (m_isArmed) {
                m_esc->stop();
            }

            m_isArmed = false;
        }

        void updateArmingFromImu(const bool imuIsLevel, const bool gyroIsCalibrating)
        {
            m_angleOkay = imuIsLevel;

            m_gyroDoneCalibrating = !gyroIsCalibrating;

            m_accDoneCalibrating = true; // XXX
        }

        bool isArmed(void)
        {
            return m_isArmed;
        }

        void attemptToArm(const uint32_t usec, const bool aux1IsSet)
        {
            static bool _doNotRepeat;

            if (aux1IsSet) {

                if (readyToArm()) {

                    if (m_isArmed) {
                        return;
                    }

                    if (!m_esc->isReady(usec)) {
                        return;
                    }

                    m_isArmed = true;
                }

            } else {

                if (m_isArmed) {
                    disarm();
                    m_isArmed = false;
                }
            }

            if (!(m_isArmed || _doNotRepeat || !readyToArm())) {
                _doNotRepeat = true;
            }
        }

        void updateFromReceiver(
                const bool throttleIsDown, const bool aux1IsSet, const bool haveSignal)
        {
            if (m_isArmed) {

                if (!haveSignal && m_haveSignal) {
                    m_gotFailsafe = true;
                    disarm();
            }
            else {
                ledSet(true);
            }
        } else {

            m_throttleIsDown = throttleIsDown;

            // If arming is disabled and the ARM switch is on
            if (!readyToArm() && aux1IsSet) {
                m_switchOkay = false;
            } else if (!aux1IsSet) {
                m_switchOkay = true;
            }

            if (!readyToArm()) {
                ledWarningFlash();
            } else {
                ledWarningDisable();
            }

            ledWarningUpdate();
        }

        m_haveSignal = haveSignal;
    }

        typedef enum {
            LED_WARNING_OFF = 0,
            LED_WARNING_ON,
            LED_WARNING_FLASH
        } ledWarningVehicleState_e;

        bool m_ledOn;

        ledWarningVehicleState_e m_ledWarningVehicleState = LED_WARNING_OFF;

        uint32_t m_ledWarningTimer = 0;

        void ledToggle(void)
        {
            m_ledOn = !m_ledOn;
            ledSet(m_ledOn);
        }

        void ledWarningRefresh(void)
        {
            switch (m_ledWarningVehicleState) {
                case LED_WARNING_OFF:
                    ledSet(false);
                    break;
                case LED_WARNING_ON:
                    ledSet(true);
                    break;
                case LED_WARNING_FLASH:
                    ledToggle();
                    break;
            }

            auto now = micros();
            m_ledWarningTimer = now + 500000;
        }

        void ledSet(bool on)
        {
            if (m_ledPin > 0) {
                digitalWrite(m_ledPin, m_ledInverted ? on : !on);
            }

            m_ledOn = on;
        }

        void ledBegin(void)
        {
            if (m_ledPin > 0) {
                pinMode(m_ledPin, OUTPUT);
            }
        }

        void ledFlash(uint8_t reps, uint16_t delayMs)
        {
            ledSet(false);
            for (auto i=0; i<reps; i++) {
                ledToggle();
                delay(delayMs);
            }
            ledSet(false);
        }

        void ledWarningFlash(void)
        {
            m_ledWarningVehicleState = LED_WARNING_FLASH;
        }

        void ledWarningDisable(void)
        {
            m_ledWarningVehicleState = LED_WARNING_OFF;
        }

        void ledWarningUpdate(void)
        {
            uint32_t now = micros();

            if ((int32_t)(now - m_ledWarningTimer) < 0) {
                return;
            }

            ledWarningRefresh();
        }

        void corePreUpdate(void) 
        {
            if (m_loopStartCycles > m_loopStartMinCycles) {
                m_loopStartCycles -= m_loopStartDeltaDownCycles;
            }
        }

        void corePostUpdate(uint32_t nowCycles)
        {
            // CPU busy
            if (intcmp(m_nextTimingCycles, nowCycles) < 0) {
                m_nextTimingCycles += m_clockRate;
            }
            lastTargetCycles = m_nextTargetCycles;
        }

        int32_t getTaskGuardCycles(void)
        {
            return m_taskGuardCycles;
        }
        
        int32_t getLoopRemainingCycles(void)
        {
            return m_loopRemainingCycles;
        }
        
        uint32_t getNextTargetCycles(void)
        {
            return m_nextTargetCycles;
        }

        bool isCoreReady(uint32_t nowCycles)
        {
            m_nextTargetCycles = lastTargetCycles + desiredPeriodCycles;

            m_loopRemainingCycles = intcmp(m_nextTargetCycles, nowCycles);

            if (m_loopRemainingCycles < -desiredPeriodCycles) {
                // A task has so grossly overrun that at entire gyro cycle has
                // been skipped This is most likely to occur when connected to
                // the configurator via USB as the serial task is
                // non-deterministic Recover as best we can, advancing
                // scheduling by a whole number of cycles
                m_nextTargetCycles += desiredPeriodCycles * (1 +
                        (m_loopRemainingCycles / -desiredPeriodCycles));
                m_loopRemainingCycles = intcmp(
                        m_nextTargetCycles, nowCycles);
            }

            // Tune out the time lost between completing the last task
            // execution and re-entering the scheduler
            if ((m_loopRemainingCycles < m_loopStartMinCycles) &&
                    (m_loopStartCycles < m_loopStartMaxCycles)) {
                m_loopStartCycles += m_loopStartDeltaUpCycles;
            }

            // Once close to the timing boundary, poll for its arrival
            return m_loopRemainingCycles < m_loopStartCycles;
        }

        bool isDynamicReady(uint32_t nowCycles) 
        {
            auto newLoopRemainingCyles =
                intcmp(m_nextTargetCycles, nowCycles);

            return newLoopRemainingCyles > m_guardMargin;
        }

        void updateDynamic(uint32_t nowCycles, uint32_t anticipatedEndCycles)
        {
            auto cyclesOverdue = intcmp(nowCycles, anticipatedEndCycles);

            if ((cyclesOverdue > 0) || (-cyclesOverdue < m_taskGuardMinCycles)) {

                if (m_taskGuardCycles < m_taskGuardMaxCycles) {
                    m_taskGuardCycles += m_taskGuardDeltaUpCycles;
                }
            } else if (m_taskGuardCycles > m_taskGuardMinCycles) {
                m_taskGuardCycles -= m_taskGuardDeltaDownCycles;
            }        
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

            m_ledPin = ledPin < 0 ? -ledPin : ledPin;
            m_ledInverted = ledPin < 0;

            imu.m_board = this;
            esc.m_board = this;
            receiver.m_board = this;

            m_loopStartCycles =
                microsecondsToClockCycles(SCHED_START_LOOP_MIN_US);
            m_loopStartMinCycles =
                microsecondsToClockCycles(SCHED_START_LOOP_MIN_US);
            m_loopStartMaxCycles =
                microsecondsToClockCycles(SCHED_START_LOOP_MAX_US);
            m_loopStartDeltaDownCycles =
                microsecondsToClockCycles(1) / SCHED_START_LOOP_DOWN_STEP;
            m_loopStartDeltaUpCycles =
                microsecondsToClockCycles(1) / SCHED_START_LOOP_UP_STEP;

            m_taskGuardMinCycles =
                microsecondsToClockCycles(TASK_GUARD_MARGIN_MIN_US);
            m_taskGuardMaxCycles =
                microsecondsToClockCycles(TASK_GUARD_MARGIN_MAX_US);
            m_taskGuardCycles = m_taskGuardMinCycles;
            m_taskGuardDeltaDownCycles =
                microsecondsToClockCycles(1) / TASK_GUARD_MARGIN_DOWN_STEP;
            m_taskGuardDeltaUpCycles =
                microsecondsToClockCycles(1) / TASK_GUARD_MARGIN_UP_STEP;

            lastTargetCycles = 0;
            m_nextTimingCycles = 0;

            desiredPeriodCycles =
                (int32_t)microsecondsToClockCycles(Clock::PERIOD());

            m_guardMargin =
                (int32_t)microsecondsToClockCycles(CHECK_GUARD_MARGIN_US);

            m_clockRate = microsecondsToClockCycles(1000000);
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

            m_attitudeTask.begin(m_imu);

            m_visualizerTask.begin(m_esc, m_receiverTask.receiver);

            m_imu->begin();

            m_esc->begin();

            ledBegin();
            ledFlash(10, 50);
        }

        void step(void)
        {
            // Realtime gyro/filtering/PID task get complete priority
            auto nowCycles = getCycleCounter();

            if (isCoreReady(nowCycles)) {
                checkCoreTasks(nowCycles);
            }

            if (isDynamicReady(getCycleCounter())) {
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

        static void printf(const char * fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            char buf[200];
            vsnprintf(buf, 200, fmt, ap); 
            outbuf(buf);
            va_end(ap);
        }

        static void reportForever(const char * fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            char buf[200];
            vsnprintf(buf, 200, fmt, ap); 
            va_end(ap);

            strcat(buf, "\n");

            while (true) {
                outbuf(buf);
                delay(500);
            }
        }

}; // class Board
