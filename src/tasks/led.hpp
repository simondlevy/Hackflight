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

#include <stdint.h>

#include <Arduino.h>

#include <safety.hpp>
#include <system.h>
#include <task.hpp>

class LedTask {

    private:

        static const uint8_t PIN = 15;

    public:

        void begin(Safety * safety) 
        {
            if (_task.didInit()){
                return;
            }

            _task.init(runLedCommsTask, "led", this, 2);

            pinMode(PIN, OUTPUT);

            set(LOW);

            _safety = safety;
        }

    private:

        static constexpr float HEARTBEAT_HZ = 1;

        static constexpr uint32_t PULSE_MSEC = 50;

        FreeRtosTask _task;

        Safety * _safety;

        static void runLedCommsTask(void * obj)
        {
            ((LedTask *)obj)->run();
        }

        void run(void)
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                if (_safety->isArmed()) { 
                    set(true);
                }
                else {
                    set(true);
                    vTaskDelay(M2T(PULSE_MSEC));
                    set(false);
                    vTaskDelayUntil(&lastWakeTime, M2T(1000/HEARTBEAT_HZ));
                }

            }
        }

        void set(const bool on)
        {
            digitalWrite(PIN, on);
        }
};
