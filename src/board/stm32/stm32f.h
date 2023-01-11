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
#include "imu.h"

class Stm32FBoard : public Stm32Board {

    private:

        AccelerometerTask m_accelerometerTask;

        void parseSkyranger(const uint8_t byte)
        {
            m_skyrangerTask.parse(byte);
        }

    protected:

        virtual void checkDynamicTasks(void) override
        {
            // STM32F boards have no auto-reset bootloader support, so we reboot
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
                    runTask(m_accelerometerTask, usec);
                    break;

                case Task::SKYRANGER:
                    runTask(m_skyrangerTask, usec);
                    break;

                default:
                    runPrioritizedTask(prioritizer, usec);
                    break;
            }
        }

        virtual void reboot(void) { }

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

        void handleSkyrangerEvent(HardwareSerial & serial)
        {
            while (serial.available()) {
                parseSkyranger(serial.read());
            }
        }

}; // class Stm32FBoard
