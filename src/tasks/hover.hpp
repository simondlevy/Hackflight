/**
 * Copyright 2025 Simon D. Levy
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

#include <debugger.hpp>
#include <ekf.hpp>
#include <opticalflow.hpp>
#include <zranger.hpp>
#include <task.hpp>

class HoverTask {

    public:

        void begin(EKF * ekf, Debugger * debugger=nullptr)
        {
            _ekf = ekf;
            _debugger = debugger;

            _zranger.init();
            _opticalflow.init();

            _task.init(runHoverTask, "hover", this, TASK_PRIORITY);
        }

    private:

        static constexpr float TASK_FREQ = 70;
        static const uint8_t TASK_PRIORITY = 3;

        static void runHoverTask(void *obj)
        {
            ((HoverTask *)obj)->run();
        }

        FreeRtosTask _task;

        EKF * _ekf;

        Debugger * _debugger;

        ZRanger _zranger;
        OpticalFlow _opticalflow;

        void run(void)
        {
            auto lastTime  = micros();

            while (true) {

                FreeRtosTask::wait(TASK_FREQ);

                _zranger.step(_ekf);
                _opticalflow.step(_ekf);
            }        
        }
};
