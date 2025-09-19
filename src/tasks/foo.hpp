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

#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>

#include <datatypes.h>
#include <lpf.hpp>
#include <m_pi.h>
#include <num.hpp>
#include <time.h>

class FooTask {

    private:

        static constexpr float CALIBRATION_PITCH = 0;
        static constexpr float CALIBRATION_ROLL = 0;

    public:

        void begin(EstimatorTask * estimatorTask,
                DebugTask * debugTask)
        {
            if (_task.didInit()) {
                return;
            }

            _estimatorTask = estimatorTask;

            _debugTask = debugTask;

            _task.init(runFooTask, "foo", this, 3);
        }

    private:

        FreeRtosTask _task;

        EstimatorTask * _estimatorTask;

        DebugTask * _debugTask;

        static void runFooTask(void *obj)
        {
            ((FooTask *)obj)->run();
        }

        void run(void)
        {
            while (true) {

                static uint32_t count;
                DebugTask::setMessage(_debugTask, "FooTask: %d", +count++);
                vTaskDelay(1);

            }
        }
};
