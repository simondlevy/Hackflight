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

#include "logic.h"
#include "esc.h"

#include <dshot.h>

class Stm32Board {

    private:

        uint8_t m_ledPin;
        bool m_ledInverted;

        uint8_t m_imuInterruptPin;

        Esc * m_esc;

        void runDynamicTasks(Logic & logic, const int16_t rawAccel[3])
        {
            if (logic.visualizerTask.gotRebootRequest()) {
                if (m_imuInterruptPin > 0) {
                    detachInterrupt(m_imuInterruptPin);
                }
                reboot();
            }

            Task::prioritizer_t prioritizer = {Task::NONE, 0};

            const uint32_t usec = micros(); 

            logic.prioritizeTasks(prioritizer, usec);

            prioritizeExtraTasks(logic, prioritizer, usec);

            switch (prioritizer.id) {

                case Task::ATTITUDE:
                    runTask(logic, logic.attitudeTask);
                    logic.updateArmingStatus(usec);
                    updateLed(logic);
                    break;

                case Task::VISUALIZER:
                    runVisualizerTask(logic);
                    break;

                case Task::RECEIVER:
                    logic.updateArmingStatus(usec);
                    updateLed(logic);
                    runTask(logic, logic.receiverTask);
                    break;

                case Task::ACCELEROMETER:
                    runTask(logic, logic.accelerometerTask);
                    logic.updateAccelerometer(rawAccel);
                    break;

                case Task::SKYRANGER:
                    runTask(logic, logic.skyrangerTask);
                    break;

                default:
                    break;
            }
        }

        void runTask(Logic & logic, Task & task)
        {
            const uint32_t anticipatedEndCycles = getAnticipatedEndCycles(logic, task);

            if (anticipatedEndCycles > 0) {

                const uint32_t usec = micros();

                task.run(usec);

                postRunTask(logic, task, usec, anticipatedEndCycles);
            } 
        }

        void postRunTask(
                Logic & logic,
                Task & task,
                const uint32_t usecStart,
                const uint32_t anticipatedEndCycles)
        {
            logic.postRunTask(
                    task, usecStart, micros(), getCycleCounter(), anticipatedEndCycles);
        }

        void updateLed(Logic & logic)
        {
            switch (logic.getArmingStatus()) {

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

        void runVisualizerTask(Logic & logic)
        {
            const uint32_t anticipatedEndCycles =
                getAnticipatedEndCycles(logic, logic.visualizerTask);

            if (anticipatedEndCycles > 0) {

                const auto usec = micros();

                while (Serial.available()) {

                    if (logic.visualizerTask.parse(Serial.read())) {
                        while (logic.mspAvailable()) {
                            Serial.write(logic.mspRead());
                        }
                    }
                }

                postRunTask(logic, logic.visualizerTask, usec, anticipatedEndCycles);
            }
        }

        uint32_t getAnticipatedEndCycles(Logic & logic, Task & task)
        {
            return logic.getAnticipatedEndCycles(task, getCycleCounter());
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

        Stm32Board(Esc & esc, const int8_t ledPin)
        {
            m_esc = &esc;

            // Support negative LED pin number for inversion
            m_ledPin = ledPin < 0 ? -ledPin : ledPin;
            m_ledInverted = ledPin < 0;
        }

        virtual void prioritizeExtraTasks(
                Logic & logic, Task::prioritizer_t & prioritizer, const uint32_t usec)
        {
            (void)logic;
            (void)prioritizer;
            (void)usec;
        }

    public:

        void setSbusValues(
                Logic & logic,
                uint16_t chanvals[],
                const uint32_t usec,
                const bool lostFrame)
        {
            logic.receiverTask.setValues(chanvals, usec, lostFrame, 172, 1811);
        }

        void setDsmxValues(
                Logic & logic,
                uint16_t chanvals[],
                const uint32_t usec,
                const bool lostFrame)
        {
            logic.receiverTask.setValues(chanvals, usec, lostFrame, 988, 2011);
        }

        void handleImuInterrupt(Logic & logic)
        {
            logic.handleImuInterrupt(getCycleCounter());
        }

        uint32_t microsToCycles(uint32_t micros)
        {
            return getClockSpeed() / 1000000 * micros;
        }

        uint32_t getCycleCounter(void)
        {
            return DWT->CYCCNT;
        }

        void begin(Logic & logic, const uint8_t imuInterruptPin, void (*irq)(void))
        {
            startCycleCounter();

            logic.begin(getClockSpeed());

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

        void step(Logic & logic, int16_t rawGyro[3], int16_t rawAccel[3])
        {
            auto nowCycles = getCycleCounter();

            if (logic.isCoreTaskReady(nowCycles)) {

                const uint32_t usec = micros();

                int32_t loopRemainingCycles = 0;

                const uint32_t nextTargetCycles =
                    logic.coreTaskPreUpdate(loopRemainingCycles);

                while (loopRemainingCycles > 0) {
                    nowCycles = getCycleCounter();
                    loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);
                }

                float mixmotors[Motors::MAX_SUPPORTED] = {};

                logic.step(rawGyro, usec, mixmotors);

                m_esc->write(
                        logic.getArmingStatus() == Logic::ARMING_ARMED ?
                        mixmotors :
                        logic.visualizerTask.motors);

                logic.updateScheduler(nowCycles, nextTargetCycles);
            }

            if (logic.isDynamicTaskReady(getCycleCounter())) {
                runDynamicTasks(logic, rawAccel);
            }
        }

        void step(
                Logic & logic,
                int16_t rawGyro[3],
                int16_t rawAccel[3],
                HardwareSerial & serial)
        {
            step(logic, rawGyro, rawAccel);

            while (logic.skyrangerTask.imuDataAvailable()) {
                serial.write(logic.skyrangerTask.readImuData());
            }
        }
};
