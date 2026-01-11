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

#include <opticalflow.hpp>
#include <zranger.hpp>
#include <tasks/estimator.hpp>

class Task2 {

    public:

        void begin(EstimatorTask * estimatorTask)
        {
            _estimatorTask = estimatorTask;

            _zranger.init();

            _opticalFlow.init();

            _task.init(runTask2, "task2", this, 2);
        }

    private:

        ZRanger _zranger;

        OpticalFlow _opticalFlow;

        static constexpr float FREQ_HZ = 50;

        static void runTask2(void * obj)
        {
            ((Task2 *)obj)->run();
        }

        FreeRtosTask _task;

        EstimatorTask * _estimatorTask;

        void run(void)
        {
            TickType_t lastWakeTime;

            lastWakeTime = xTaskGetTickCount();

            while (true) {

                vTaskDelayUntil(&lastWakeTime, 1000/FREQ_HZ);

                tofMeasurement_t tofData = {};
                if (_zranger.read(tofData, xTaskGetTickCount())) {
                    _estimatorTask->enqueueRange(&tofData);
                }

                flowMeasurement_t flowData = {};
                if (_opticalFlow.read(flowData)) {
                    _estimatorTask->enqueueFlow(&flowData);
                }
            }
        }
};
