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

#include <__control__.hpp>
#include <safety.hpp>
#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/setpoint.hpp>

class CoreTask {

    public:

        void begin(
                ClosedLoopControl * closedLoopControl,
                Safety * safety,
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                SetpointTask * setpointTask,
                const uint8_t motorCount,
                const mixFun_t mixFun,
				DebugTask * debugTask=nullptr)
        {
            _safety = safety;

            _imuTask = imuTask;
            _debugTask = debugTask;

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static void runCoreTask(void *arg)
        {
            ((CoreTask *)arg)->run();
        }

        FreeRtosTask _task;

        Safety * _safety;

        DebugTask * _debugTask;
        ImuTask * _imuTask;

        void run()
        {
            for (uint32_t step=1; ; step++) {

                // Wait for IMU
                _imuTask->waitDataReady();

                const auto status = _safety->getStatus();

                if (status == Safety::LOST_CONTACT) {
                    // No way to recover from this
                }

                else switch (status) {

                    case Safety::IDLE:
                        DebugTask::setMessage(_debugTask, "idle");
                        break;

                    case Safety::ARMED:
                        DebugTask::setMessage(_debugTask, "armed");
                        break;

                    case Safety::FLYING:
                        DebugTask::setMessage(_debugTask, "flying");
                        break;

                    case Safety::LANDING:
                        DebugTask::setMessage(_debugTask, "landing");
                        break;
                }
            }
        }

        // Device-dependent ---------------------------

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
