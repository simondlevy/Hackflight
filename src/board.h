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

#include "logic/logic.h"
#include "esc.h"

#include <dshot.h>

class Stm32Board {

    private:

        uint8_t m_ledPin;
        bool m_ledInverted;

        uint8_t m_imuInterruptPin;

        Esc * m_esc;

        uint32_t getCycleCounter(void)
        {
            return DWT->CYCCNT;
        }

        bool runDynamicTasks(Logic & logic, const int16_t rawAccel[3], const uint32_t usec)
        {
            auto taskId = logic.prioritizeTasks(rawAccel, usec);

            if (taskId != Task::NONE) {

                const auto anticipatedEndCycles = 
                    logic.getAnticipatedEndCycles(getCycleCounter(), taskId);

                if (anticipatedEndCycles > 0) {

                    const uint32_t usec = micros();

                    if (taskId == Task::VISUALIZER) {
                        runVisualizerTask(logic);
                    }
                    else {
                        logic.runTask(taskId, usec);
                    }

                    logic.postRunTask(
                            taskId,
                            usec,
                            micros(),
                            getCycleCounter(),
                            anticipatedEndCycles);
                }
            }

            // LED udpate needed?
            return taskId == Task::ATTITUDE || taskId == Task::RECEIVER;
        }

        void runVisualizerTask(Logic & logic)
        {
            while (Serial.available()) {

                if (logic.mspParse(Serial.read())) {
                    while (logic.mspBytesAvailable()) {
                        Serial.write(logic.mspGetByte());
                    }
                }
            }
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

    public:

        void handleImuInterrupt(Logic & logic)
        {
            logic.handleImuInterrupt(getCycleCounter());
        }

        void begin(Logic & logic)
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

                m_esc->write(logic.getMotors(rawGyro, usec));

                logic.updateScheduler(nowCycles, nextTargetCycles);
            }

            if (logic.isDynamicTaskReady(getCycleCounter())) {

                if (logic.gotRebootRequest()) {
                    if (m_imuInterruptPin > 0) {
                        detachInterrupt(m_imuInterruptPin);
                    }
                    reboot();
                }

                if (runDynamicTasks(logic, rawAccel, micros())) {
                    updateLed(logic);
                }
            }
        }

        void step(
                Logic & logic,
                int16_t rawGyro[3],
                int16_t rawAccel[3],
                HardwareSerial & serial)
        {
            step(logic, rawGyro, rawAccel);

            while (logic.skyrangerDataAvailable()) {
                serial.write(logic.readSkyrangerData());
            }
        }

        void setImuInterrupt(
                const uint8_t pin, void (*irq)(void), const uint32_t mode)
        {
            pinMode(pin, INPUT);
            attachInterrupt(pin, irq, mode);  

            // Store pin for call to detachInterrupt() for reboot
            m_imuInterruptPin = pin;
        }
};
