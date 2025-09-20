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

#include <safety.hpp>
#include <task.hpp>
#include <tasks/imu.hpp>

class LedTask {

    public:

        void begin(
                Safety * safety,
                ImuTask * imuTask,
                const uint8_t pin,
                const bool active_low) 
        {
            if (_task.didInit()){
                return;
            }

            _pin = pin;

            _active_low = active_low;

            _imuTask = imuTask;

            _task.init(runLedTask, "led", this, 2);

            pinMode(_pin, OUTPUT);

            set(LOW);

            _safety = safety;
        }

    private:

        static constexpr float HEARTBEAT_HZ = 1;

        static constexpr float IMU_CALIBRATION_HZ = 3;

        static constexpr uint32_t PULSE_MSEC = 50;

        uint8_t _pin;

        bool _active_low;

        FreeRtosTask _task;

        Safety * _safety;

        ImuTask * _imuTask;

        static void runLedTask(void * obj)
        {
            ((LedTask *)obj)->run();
        }

        void run(void)
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                if (!_imuTask->imuIsCalibrated()) {
                    blink(lastWakeTime, IMU_CALIBRATION_HZ);
                }

                else if (_safety->isArmed()) { 
                    set(true);
                }
                else {
                    blink(lastWakeTime, HEARTBEAT_HZ);
                }

            }
        }

        void set(const bool on)
        {
            digitalWrite(_pin, _active_low ? !on : on);
        }

        void blink(TickType_t & lastWakeTime, const float rate)
        {
            set(true);
            vTaskDelay(PULSE_MSEC);
            set(false);
            vTaskDelayUntil(&lastWakeTime, 1000/rate);
        }
};
