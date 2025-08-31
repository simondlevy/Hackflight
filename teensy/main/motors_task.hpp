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

#include <task.hpp>

#include <Arduino.h>

#include <oneshot125.hpp>
#include <vector>

class MotorsTask {

    private:

        // XXX not sure why we have to do this
        static const uint8_t M1_OFFSET = 22;
        static const uint8_t M2_OFFSET = 0;
        static const uint8_t M3_OFFSET = 22;
        static const uint8_t M4_OFFSET = 0;

        static constexpr float FREQ_HZ = 1000;

        std::vector<uint8_t> PINS = {2, 23, 14, 9};

    public:

        void begin() 
        {
            if (_task.didInit()){
                return;
            }

            _task.init(runMotorsTask, "motors", this, 5);
        }

    private:

        OneShot125 motors = OneShot125(PINS);

        FreeRtosTask _task;

        static void runMotorsTask(void * obj)
        {
            ((MotorsTask *)obj)->run();
        }

        void run(void)
        {
            motors.arm();

            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelayUntil(&lastWakeTime, M2T(1000/FREQ_HZ));

                const uint8_t pulseWidth = 125;

                motors.set(0, pulseWidth + M1_OFFSET);
                motors.set(1, pulseWidth + M2_OFFSET);
                motors.set(2, pulseWidth + M3_OFFSET);
                motors.set(3, pulseWidth + M4_OFFSET);

                motors.run();
            }
        }
};
