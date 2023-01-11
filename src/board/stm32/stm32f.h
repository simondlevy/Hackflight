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


    protected:

        virtual void checkDynamicTasks(void) override
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

        virtual void reboot(void) { }

}; // class Stm32FBoard
