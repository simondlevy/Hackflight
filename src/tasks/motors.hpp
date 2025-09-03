/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

class MotorsTask {

    public:

        void begin(void)
        {
            _task.init(runMotorsTask, "motors", this, 5);

            device_init();

            device_stop();
        }

        int getRatio(uint32_t id)
        {
            return ratios[id];
        }

        void stop()
        {
            device_stop();
        }

        void setRatios(const uint16_t ratios[])
        {
            setRatio(0, ratios[0]);
            setRatio(1, ratios[1]);
            setRatio(2, ratios[2]);
            setRatio(3, ratios[3]);
        }

    private:

        static const uint32_t FREQ_HZ = 1000;

        FreeRtosTask _task;

        uint32_t ratios[4]; 

        static void runMotorsTask(void * obj)
        {
            ((MotorsTask *)obj)->run();
        }

        void run(void)
        {
            TickType_t lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelayUntil(&lastWakeTime, 1000/FREQ_HZ);

            }
        }

        void setRatio(uint32_t id, uint16_t ratio)
        {
            ratios[id] = ratio;

            device_setRatio(id, ratio);
        }

        void device_init();

        void device_setRatio(const uint32_t, const uint16_t ratio);

        void device_stop();
};
