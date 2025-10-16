/**
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Arduino.h>

#include <stdint.h>

#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/imu.hpp>

class LedTask {

    public:

        void begin(ImuTask * imuTask, DebugTask * debugTask=nullptr)
        {
            _imuTask = imuTask;

            _debugTask = debugTask;

            _task.init(runLedTask, "led", this, 2);

            device_init();
        }

        void setArmed(const bool armed)
        {
            _armed = armed;
        }

    private:

        static constexpr float HEARTBEAT_HZ = 1;

        static constexpr float IMU_CALIBRATION_HZ = 3;

        static constexpr uint32_t PULSE_MSEC = 50;

        FreeRtosTask _task;

        ImuTask * _imuTask;

        DebugTask * _debugTask;

        bool _armed;

        static void runLedTask(void * obj)
        {
            ((LedTask *)obj)->run();
        }

        void run(void)
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelay(1);

                if (!_imuTask->imuIsCalibrated()) {
                    blink(lastWakeTime, IMU_CALIBRATION_HZ);
                    DebugTask::setMessage(_debugTask, "calibrating");
                }

                else if (_armed) { 
                    device_set(true);
                    DebugTask::setMessage(_debugTask, "armed");
                }
                else {
                    blink(lastWakeTime, HEARTBEAT_HZ);
                    DebugTask::setMessage(_debugTask, "heartbeat");
                }

            }
        }


        void blink(TickType_t & lastWakeTime, const float rate)
        {
            device_set(true);
            vTaskDelay(PULSE_MSEC);
            device_set(false);
            vTaskDelayUntil(&lastWakeTime, 1000/rate);
        }

        void device_init();

        void device_set(const bool on);
};
