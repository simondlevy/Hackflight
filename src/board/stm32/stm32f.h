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

#include <Wire.h>

#include "board/stm32.h"
#include "task/accelerometer.h"
#include "imu.h"

class Stm32FBoard : public Stm32Board {

    private:

        AccelerometerTask m_accelerometerTask; 

        void parseSkyranger(const uint8_t byte)
        {
            m_skyrangerTask.parse(byte);
        }

        void runAccelerometerTask(void)
        {
            const auto nextTargetCycles = m_nextTargetCycles;
            const auto taskRequiredTimeUs = m_accelerometerTask.getRequiredTime();
            const auto nowCycles = getCycleCounter();
            const auto loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);

            // Allow a little extra time
            const auto taskRequiredCycles =
                (int32_t)microsecondsToClockCycles((uint32_t)taskRequiredTimeUs) +
                getTaskGuardCycles();

            if (taskRequiredCycles < loopRemainingCycles) {

                const auto anticipatedEndCycles = nowCycles + taskRequiredCycles;

                const auto usec = micros();

                m_accelerometerTask.run();

                m_accelerometerTask.update(usec, micros()-usec);

                updateDynamic(getCycleCounter(), anticipatedEndCycles);
            } else {
                m_accelerometerTask.enableRun();
            }
        }

        void runSkyrangerTask(void)
        {
            const auto nextTargetCycles = m_nextTargetCycles;
            const auto taskRequiredTimeUs = m_skyrangerTask.getRequiredTime();
            const auto nowCycles = getCycleCounter();
            const auto loopRemainingCycles = intcmp(nextTargetCycles, nowCycles);

            // Allow a little extra time
            const auto taskRequiredCycles =
                (int32_t)microsecondsToClockCycles((uint32_t)taskRequiredTimeUs) +
                getTaskGuardCycles();

            if (taskRequiredCycles < loopRemainingCycles) {

                const auto anticipatedEndCycles = nowCycles + taskRequiredCycles;

                const auto usec = micros();

                m_skyrangerTask.run();

                m_skyrangerTask.update(usec, micros()-usec);

                updateDynamic(getCycleCounter(), anticipatedEndCycles);
            } else {
                m_skyrangerTask.enableRun();
            }
        }

    protected:

        // STM32F boards have no auto-reset bootloader support, so we reboot on
        // an external input
        virtual void reboot(void) { }

        virtual void checkDynamicTasks(void) override
        {
            if (m_visualizerTask.gotRebootRequest()) {
                reboot();
            }

            const uint32_t usec = micros();

            // Prioritize primary dynamic tasks (attitude, visualizer, receiver)
            Task::prioritizer_t prioritizer = prioritizeDynamicTasks(usec);

            // Add in accelerometer and skyranger tasks
            m_accelerometerTask.prioritize(usec, prioritizer);
            m_skyrangerTask.prioritize(usec, prioritizer);

            switch (prioritizer.id) {

                case Task::ACCELEROMETER:
                    runAccelerometerTask();
                    break;

                case Task::SKYRANGER:
                    runSkyrangerTask();
                    break;

                default:
                    runPrioritizedTask(prioritizer);
                    break;
            }
        }

    public:

        Stm32FBoard(
                Receiver & receiver,
                Imu & imu,
                vector<PidController *> & pids,
                Mixer & mixer,
                Esc & esc,
                const uint8_t ledPin) 
            : Stm32Board(receiver, imu, pids, mixer, esc, ledPin)
        {
        }

        void begin(void)
        {
            Board::begin();

            m_accelerometerTask.begin(m_imu);
        }

        void handleSkyrangerEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                parseSkyranger(serial.read());
            }
        }

}; // class Stm32FBoard
