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

#include "core/mixer.h"
#include "logic.h"
#include "esc.h"

#include <dshot.h>

class Stm32Board {

    private:

        uint8_t m_ledPin;
        bool m_ledInverted;

        uint8_t m_imuInterruptPin;

        void runDynamicTasks(Imu & imu, const int16_t rawAccel[3])
        {
            if (m_logic.gotRebootRequest()) {
                if (m_imuInterruptPin > 0) {
                    detachInterrupt(m_imuInterruptPin);
                }
                reboot();
            }

            Task::prioritizer_t prioritizer = {Task::NONE, 0};

            const uint32_t usec = micros(); 

            m_logic.prioritizeTasks(prioritizer, usec);

            prioritizeExtraTasks(prioritizer, usec);

            switch (prioritizer.id) {

                case Task::ATTITUDE:
                    runTask(prioritizer.id);
                    m_logic.updateArmingStatus(imu, usec);
                    updateLed();
                    break;

                case Task::VISUALIZER:
                    runVisualizerTask();
                    break;

                case Task::RECEIVER:
                    m_logic.updateArmingStatus(imu, usec);
                    updateLed();
                    runTask(prioritizer.id);
                    break;

                case Task::ACCELEROMETER:
                    runTask(prioritizer.id);
                    m_logic.updateAccelerometer(imu, rawAccel);
                    break;

                case Task::SKYRANGER:
                    runTask(prioritizer.id);
                    break;

                default:
                    break;
            }
        }

        void runTask(Task::id_e id)
        {
            const uint32_t anticipatedEndCycles = getTaskAnticipatedEndCycles(id);

            if (anticipatedEndCycles > 0) {

                const uint32_t usec = micros();

                m_logic.runTask(id, usec);

                postRunTask(id, usec, anticipatedEndCycles);
            } 
        }

        void postRunTask(
                Task::id_e id,
                const uint32_t usecStart,
                const uint32_t anticipatedEndCycles)
        {
            m_logic.postRunTask(
                    id, usecStart, micros(), getCycleCounter(), anticipatedEndCycles);
        }

        void updateLed(void)
        {
            switch (m_logic.getArmingStatus()) {

                case Logic::ARMING_UNREADY:
                    ledBlink(500);
                    break;

                case Logic::ARMING_READY:
                    ledSet(false);
                    break;

                case Logic::ARMING_ARMED:
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
            digitalWrite(m_ledPin, m_ledInverted ? on : !on);
        }

        // STM32F boards have no auto-reset bootloader support, so we reboot on
        // an external input
        virtual void reboot(void)
        {
        }

        void runVisualizerTask(void)
        {
            const uint32_t anticipatedEndCycles = 
                getTaskAnticipatedEndCycles(Task::VISUALIZER);

            if (anticipatedEndCycles > 0) {

                const auto usec = micros();

                while (Serial.available()) {

                    if (m_logic.mspParse(Serial.read())) {
                        while (m_logic.mspAvailable()) {
                            Serial.write(m_logic.mspRead());
                        }
                    }
                }

                postRunTask(Task::VISUALIZER, usec, anticipatedEndCycles);
            }
        }

        uint32_t getTaskAnticipatedEndCycles(Task::id_e id)
        {
            return m_logic.getTaskAnticipatedEndCycles(id, getCycleCounter());
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

    protected:

        Logic m_logic;

        Stm32Board(const int8_t ledPin)
        {
            // Support negative LED pin number for inversion
            m_ledPin = ledPin < 0 ? -ledPin : ledPin;
            m_ledInverted = ledPin < 0;
        }

        virtual void prioritizeExtraTasks(
                Task::prioritizer_t & prioritizer, const uint32_t usec)
        {
            (void)prioritizer;
            (void)usec;
        }

    public:

        void setSbusValues(uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            m_logic.setSbusValues(chanvals, usec, lostFrame);
        }

        void setDsmxValues(uint16_t chanvals[], const uint32_t usec, const bool lostFrame)
        {
            m_logic.setDsmxValues(chanvals, usec, lostFrame);
        }

        void handleImuInterrupt(Imu & imu)
        {
            m_logic.handleImuInterrupt(imu, getCycleCounter());
        }

        uint32_t microsToCycles(uint32_t micros)
        {
            return getClockSpeed() / 1000000 * micros;
        }

        uint32_t getCycleCounter(void)
        {
            return DWT->CYCCNT;
        }

        void begin(Imu & imu, const uint8_t imuInterruptPin, void (*irq)(void))
        {
            startCycleCounter();

            m_logic.begin(imu, getClockSpeed());

            pinMode(m_ledPin, OUTPUT);

            ledSet(false);
            for (auto i=0; i<10; i++) {
                static bool ledOn;
                ledOn = !ledOn;
                ledSet(ledOn);
                delay(50);
            }
            ledSet(false);

            pinMode(imuInterruptPin, INPUT);
            attachInterrupt(imuInterruptPin, irq, RISING);  

            // Store pin for call to detachInterrupt() for reboot
            m_imuInterruptPin = imuInterruptPin;
        }

        void step(
                Imu & imu,
                std::vector<PidController *> pids,
                Mixer & mixer,
                Esc & esc,
                int16_t rawGyro[3],
                int16_t rawAccel[3])
        {
            auto nowCycles = getCycleCounter();

            if (m_logic.isCoreTaskReady(nowCycles)) {

                const uint32_t usec = micros();

                int32_t loopRemainingCycles = 0;

                const uint32_t nextTargetCycles =
                    m_logic.coreTaskPreUpdate(loopRemainingCycles);

                while (loopRemainingCycles > 0) {
                    nowCycles = getCycleCounter();
                    loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);
                }

                float mixmotors[Mixer::MAX_MOTORS] = {};

                // Wait a little for DSHOT ESCs to start up
                if (esc.isReady(usec)) {
                    m_logic.step(imu, pids, mixer, rawGyro, usec, mixmotors);
                }

                esc.write(
                        m_logic.getArmingStatus() == Logic::ARMING_ARMED ?
                        mixmotors :
                        m_logic.getVisualizerMotors());

                m_logic.updateScheduler(imu, nowCycles, nextTargetCycles);
            }

            if (m_logic.isDynamicTaskReady(getCycleCounter())) {
                runDynamicTasks(imu, rawAccel);
            }
        }

        void step(
                Imu & imu,
                std::vector<PidController *> pids,
                Mixer & mixer,
                Esc & esc,
                int16_t rawGyro[3],
                int16_t rawAccel[3],
                HardwareSerial & serial)
        {
            step(imu, pids, mixer, esc, rawGyro, rawAccel);

            while (m_logic.skyrangerTask.imuDataAvailable()) {
                serial.write(m_logic.skyrangerTask.readImuData());
            }
        }
};
