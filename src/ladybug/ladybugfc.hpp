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

#include <Arduino.h>
#include <Wire.h>

#include <usfs.hpp>

#include <hackflight.hpp>
#include <ladybug/escs/brushed.hpp>
#include <ladybug/mixer.hpp>
#include <ladybug/scheduler.hpp>
#include <ladybug/tasks/estimator.hpp>
#include <ladybug/tasks/receiver.hpp>
#include <ladybug/tasks/visualizer.hpp>

class LadybugFC {

    public:

        void begin(void (*isr)(void))
        {
            Serial.begin(115200);

            Wire.begin();
            Wire.setClock(400000); 
            delay(100);

            startCycleCounter();

            pinMode(LED_PIN, OUTPUT);

            ledSet(false);
            for (auto i=0; i<10; i++) {
                static bool ledOn;
                ledOn = !ledOn;
                ledSet(ledOn);
                delay(50);
            }
            ledSet(false);

            pinMode(IMU_INTERRUPT_PIN, INPUT);
            attachInterrupt(IMU_INTERRUPT_PIN, isr, RISING);  

            _usfs.loadFirmware(); 

            _usfs.begin(
                    ACCEL_BANDWIDTH,
                    GYRO_BANDWIDTH,
                    QUAT_DIVISOR,
                    MAG_RATE,
                    ACCEL_RATE_TENTH,
                    GYRO_RATE_TENTH,
                    BARO_RATE,
                    INTERRUPT_ENABLE);

            // Clear interrupts
            Usfs::checkStatus();

            _esc.begin();

            _estimatorTask.begin();
        }

        void handleImuInterrupt(void)
        {
            _imuInterruptCount++;
            _gotNewImuData = true;
        }

        void step(std::vector<PidController *> pids, Mixer & mixer)
        {
            // Get state vector angular velocities directly from gyro
            Hackflight::gyroToVehicleState(_gyro, _state);

            if (_gotNewImuData) { 

                _gotNewImuData = false;  

                uint8_t eventStatus = Usfs::checkStatus(); 

                if (Usfs::eventStatusIsError(eventStatus)) { 
                    Usfs::reportError(eventStatus);
                }

                if (Usfs::eventStatusIsGyrometer(eventStatus)) { 
                    _usfs.readGyrometerScaled(_gyro.x, _gyro.y, _gyro.z);
                    _gyro.y = -_gyro.y; // negate for nose-down positive
                    _gyro.z = -_gyro.z; // negate for nose-left positive
                }

                if (Usfs::eventStatusIsQuaternion(eventStatus)) { 
                    _usfs.readQuaternion(_quat.w, _quat.x, _quat.y, _quat.z);
                }
            } 

            auto nowCycles = getCycleCounter();

            if (isCoreTaskReady(nowCycles)) {

                const uint32_t usec = micros();

                int32_t loopRemainingCycles = 0;

                const uint32_t nextTargetCycles =
                    _scheduler.corePreUpdate(loopRemainingCycles);

                while (loopRemainingCycles > 0) {
                    nowCycles = getCycleCounter();
                    loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);
                }

                float mixmotors[Hackflight::MAX_MOTOR_COUNT] = {};

                if (_esc.isReady(usec)) {

                    demands_t demands = {0, 0, 0, 0};

                    _receiverTask.getDemands(demands);

                    auto pidReset = _receiverTask.throttleIsDown();

                    PidController::run(pids, demands, _state, usec, pidReset);

                    mixer.getMotors(demands, mixmotors);
                }

                _esc.write(
                        getArmingStatus() == ARMING_ARMED ?
                        mixmotors :
                        getVisualizerMotors());

                updateScheduler(nowCycles, nextTargetCycles);
            }

            if (_scheduler.isDynamicReady(getCycleCounter())) {
                runDynamicTasks();
            }
        }

        void setDsmxValues(
                uint16_t chanvals[],
                const uint32_t usec,
                const bool lostFrame)
        {
            _receiverTask.setValues(chanvals, usec, lostFrame, 988, 2011);
        }

        void setSbusValues(
                uint16_t chanvals[],
                const uint32_t usec,
                const bool lostFrame)
        {
            _receiverTask.setValues(chanvals, usec, lostFrame, 172, 1811);
        }

    private:

        // Arbitrary
        static const uint8_t  ACCEL_BANDWIDTH  = 3;
        static const uint8_t  GYRO_BANDWIDTH   = 3;
        static const uint8_t  QUAT_DIVISOR     = 1;
        static const uint8_t  MAG_RATE         = 100;
        static const uint8_t  ACCEL_RATE_TENTH = 20; // Multiply by 10 to get actual rate
        static const uint8_t  BARO_RATE        = 50;
        static const uint16_t ACCEL_SCALE      = 8;
        static const uint16_t MAG_SCALE        = 1000;

        typedef enum {

            ARMING_UNREADY,
            ARMING_READY,
            ARMING_ARMED,
            ARMING_FAILSAFE

        } armingStatus_e;

        static const uint8_t  GYRO_RATE_TENTH = 100;   // 1/10th actual rate

        static const uint8_t IMU_INTERRUPT_PIN = 0x0C;

        static const uint8_t INTERRUPT_ENABLE = Usfs::INTERRUPT_RESET_REQUIRED |
            Usfs::INTERRUPT_ERROR |
            Usfs::INTERRUPT_GYRO | 
            Usfs::INTERRUPT_QUAT;

        // Gyro interrupt counts over which to measure loop time and skew
        static const uint32_t GYRO_RATE_COUNT = 25000;
        static const uint32_t GYRO_LOCK_COUNT = 400;

        static constexpr float MAX_ARMING_ANGLE_DEG = 25;

        bool _gotNewImuData;

        Scheduler _scheduler;

        armingStatus_e _armingStatus;

        vehicleState_t _state;

        Msp _msp;

        EstimatorTask      _estimatorTask;
        ReceiverTask      _receiverTask;
        VisualizerTask    _visualizerTask; 

        std::vector<uint8_t> MOTOR_PINS = {0x0D, 0x10, 0x03, 0x0B};

        quaternion_t _quat;

        Usfs _usfs;

        Axis3f _gyro;

        uint32_t _gyroSyncTime;

        uint32_t _imuInterruptCount;

        BrushedEsc _esc = BrushedEsc(&MOTOR_PINS);

        int16_t _rawAccel[3] = {};

        static const uint8_t LED_PIN = 0x12;

        uint32_t microsToCycles(uint32_t micros)
        {
            return getClockSpeed() / 1000000 * micros;
        }

        uint32_t getCycleCounter(void)
        {
            return DWT->CYCCNT;
        }

        void postRunTask(
                LadybugTask::id_e id,
                const uint32_t usecStart,
                const uint32_t usecEnd,
                const uint32_t nowCycles,
                const uint32_t anticipatedEndCycles)
        {
            const auto usecTaken = usecEnd-usecStart;

            switch (id) {

                case LadybugTask::ESTIMATOR:
                    _estimatorTask.update(usecStart, usecTaken);
                    break;

                case LadybugTask::VISUALIZER:
                    _visualizerTask.update(usecStart, usecTaken);
                    break;

                case LadybugTask::RECEIVER:
                    _receiverTask.update(usecStart, usecTaken);
                    break;

                default:
                    break;
            }

            _scheduler.updateDynamic(nowCycles, anticipatedEndCycles);
        }


        void runDynamicTasks(void)
        {
            LadybugTask::prioritizer_t prioritizer = {LadybugTask::NONE, 0};

            const uint32_t usec = micros(); 

            prioritizeTasks(prioritizer, usec);

            switch (prioritizer.id) {

                case LadybugTask::ESTIMATOR:
                    runTask(prioritizer.id);
                    updateArmingStatus(usec);
                    updateLed();
                    break;

                case LadybugTask::VISUALIZER:
                    runVisualizerTask();
                    break;

                case LadybugTask::RECEIVER:
                    updateArmingStatus(usec);
                    updateLed();
                    runTask(prioritizer.id);
                    break;

                default:
                    break;
            }
        }

        void runTask(LadybugTask::id_e id)
        {
            const uint32_t anticipatedEndCycles = getTaskAnticipatedEndCycles(id);

            if (anticipatedEndCycles > 0) {

                const uint32_t usec = micros();

                switch (id) {

                    case LadybugTask::ESTIMATOR:
                        _estimatorTask.run(_quat, _state);
                        break;

                    case LadybugTask::RECEIVER:
                        _receiverTask.run();
                        break;

                    default:
                        break;
                }

                postRunTask(id, usec, anticipatedEndCycles);
            } 
        }

        void postRunTask(
                LadybugTask::id_e id,
                const uint32_t usecStart,
                const uint32_t anticipatedEndCycles)
        {
            postRunTask(
                    id, usecStart, micros(), getCycleCounter(), anticipatedEndCycles);
        }

        void updateLed(void)
        {
            switch (getArmingStatus()) {

                case ARMING_UNREADY:
                    ledBlink(500);
                    break;

                case ARMING_READY:
                    ledSet(false);
                    break;

                case ARMING_ARMED:
                    ledSet(true);
                    break;

                default: // failsafe
                    ledBlink(200);
                    break;
            }
        }

        void ledBlink(const uint32_t msecDelay)
        {
            static bool ledPrev;
            static uint32_t msecPrev;
            const uint32_t msecCurr = millis();

            if (msecCurr - msecPrev > msecDelay) {
                ledPrev = !ledPrev;
                ledSet(ledPrev);
                msecPrev = msecCurr;
            }
        }


        void ledSet(bool on)
        {
            digitalWrite(LED_PIN, on);
        }

        void runVisualizerTask(void)
        {
            const uint32_t anticipatedEndCycles = 
                getTaskAnticipatedEndCycles(LadybugTask::VISUALIZER);

            if (anticipatedEndCycles > 0) {

                const auto usec = micros();

                while (Serial.available()) {

                    if (mspParse(Serial.read())) {
                        while (mspAvailable()) {
                            Serial.write(mspRead());
                        }
                    }
                }

                postRunTask(LadybugTask::VISUALIZER, usec, anticipatedEndCycles);
            }
        }

        uint32_t getTaskAnticipatedEndCycles(LadybugTask::id_e id)
        {
            return getTaskAnticipatedEndCycles(id, getCycleCounter());
        }

        uint32_t getClockSpeed(void) 
        {
            return SystemCoreClock;
        }

        void startCycleCounter(void)
        {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

            __O uint32_t *DWTLAR = (uint32_t *)(DWT_BASE + 0x0FB0);
            *(DWTLAR) = 0xC5ACCE55;

            DWT->CYCCNT = 0;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }

        uint32_t getTaskAnticipatedEndCycles(LadybugTask::id_e id, const uint32_t nowCycles)
        {
            uint32_t endCycles = 0;

            switch (id) {

                case LadybugTask::ESTIMATOR:
                    endCycles = _scheduler.getAnticipatedEndCycles(
                            _estimatorTask, nowCycles);
                    break;

                case LadybugTask::VISUALIZER:
                    endCycles = _scheduler.getAnticipatedEndCycles(
                            _visualizerTask, nowCycles);
                    break;

                case LadybugTask::RECEIVER:
                    endCycles = _scheduler.getAnticipatedEndCycles(
                            _receiverTask, nowCycles);
                    break;

                default:
                    break;
            }

            return endCycles;
        }

        bool isCoreTaskReady(const uint32_t nowCycles)
        {
            return _scheduler.isCoreReady(nowCycles);
        }

        void updateScheduler(
                const uint32_t nowCycles,
                const uint32_t nextTargetCycles)
        {
            _scheduler.corePostUpdate(nowCycles);

            // Bring the _scheduler into lock with the gyro Track the actual
            // gyro rate over given number of cycle times and set the expected
            // timebase
            static uint32_t _terminalGyroRateCount;
            static int32_t _sampleRateStartCycles;

            if ((_terminalGyroRateCount == 0)) {
                _terminalGyroRateCount = _imuInterruptCount + GYRO_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
            }

            if (_imuInterruptCount >= _terminalGyroRateCount) {
                // Calculate number of clock cycles on average between gyro
                // interrupts
                uint32_t sampleCycles = nowCycles - _sampleRateStartCycles;
                _scheduler.desiredPeriodCycles = sampleCycles / GYRO_RATE_COUNT;
                _sampleRateStartCycles = nowCycles;
                _terminalGyroRateCount += GYRO_RATE_COUNT;
            }

            // Track actual gyro rate over given number of cycle times and
            // remove skew
            static uint32_t _terminalGyroLockCount;
            static int32_t _gyroSkewAccum;

            auto gyroSkew =
                getGyroSkew(nextTargetCycles, _scheduler.desiredPeriodCycles);

            _gyroSkewAccum += gyroSkew;

            if ((_terminalGyroLockCount == 0)) {
                _terminalGyroLockCount = _imuInterruptCount + GYRO_LOCK_COUNT;
            }

            if (_imuInterruptCount >= _terminalGyroLockCount) {
                _terminalGyroLockCount += GYRO_LOCK_COUNT;

                // Move the desired start time of the gyroSampleTask
                _scheduler.lastTargetCycles -= (_gyroSkewAccum/GYRO_LOCK_COUNT);

                _gyroSkewAccum = 0;
            }
        }

        void prioritizeTasks(LadybugTask::prioritizer_t & prioritizer, const uint32_t usec)
        {
            _receiverTask.prioritize(usec, prioritizer);
            _estimatorTask.prioritize(usec, prioritizer);
            _visualizerTask.prioritize(usec, prioritizer);
        }

        uint8_t mspAvailable(void)
        {
            return _msp.available();
        }

        uint8_t mspRead(void)
        {
            return _msp.read();
        }

        bool mspParse(const uint8_t byte)
        {
            return _visualizerTask.parse(_state, _receiverTask, _msp, byte);
        }

        float * getVisualizerMotors(void)
        {
            return _visualizerTask.motors;
        }

        void checkFailsafe(const uint32_t usec)
        {
            static bool hadSignal;

            const auto haveSignal = _receiverTask.haveSignal(usec);

            if (haveSignal) {
                hadSignal = true;
            }

            if (hadSignal && !haveSignal) {
                _armingStatus = ARMING_FAILSAFE;
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

            const auto imuIsLevel =
                fabsf(_state.phi) < MAX_ARMING_ANGLE_DEG &&
                fabsf(_state.theta) < MAX_ARMING_ANGLE_DEG;

            const auto gyroDoneCalibrating = true; // XXX

            const auto haveReceiverSignal = _receiverTask.haveSignal(usec);

            return
                auxSwitchWasOff &&
                gyroDoneCalibrating &&
                imuIsLevel &&
                _receiverTask.throttleIsDown() &&
                haveReceiverSignal;
        }

        float getAux1(void)
        {
            return _receiverTask.getRawAux1();
        }

        void checkArmingSwitch(void)
        {
            static bool aux1WasSet;

            if (getAux1() > 1500) {
                if (!aux1WasSet) {
                    _armingStatus = ARMING_ARMED;
                }
                aux1WasSet = true;
            }
            else {
                if (aux1WasSet) {
                    _armingStatus = ARMING_READY;
                }
                aux1WasSet = false;
            }
        }

        armingStatus_e getArmingStatus(void)
        {
            return _armingStatus;
        }

        void updateArmingStatus(const uint32_t usec)
        {
            checkFailsafe(usec);

            switch (_armingStatus) {

                case ARMING_UNREADY:
                    if (safeToArm(usec)) {
                        _armingStatus = ARMING_READY;
                    }
                    break;

                case ARMING_READY:
                    if (safeToArm(usec)) {
                        checkArmingSwitch();
                    }
                    else {
                        _armingStatus = ARMING_UNREADY;
                    }
                    break;

                case ARMING_ARMED:
                    checkArmingSwitch();
                    break;

                default: // failsafe
                    break;
            }
        }

        int32_t getGyroSkew(
                const uint32_t nextTargetCycles,
                const int32_t desiredPeriodCycles)
        {
            const auto skew =
                intcmp(nextTargetCycles, _gyroSyncTime) % desiredPeriodCycles;

            return skew > (desiredPeriodCycles / 2) ? skew - desiredPeriodCycles : skew;
        }

}; // class LadybugFC
