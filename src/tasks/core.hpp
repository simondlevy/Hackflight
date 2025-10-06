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

#include <task.hpp>
#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <tasks/setpoint.hpp>

class CoreTask {

    public:

        void begin(
                ClosedLoopControl * closedLoopControl,
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                SetpointTask * setpointTask,
                const uint8_t motorCount,
                const mixFun_t mixFun,
				DebugTask * debugTask=nullptr)
        {
            _imuTask = imuTask;
            _debugTask = debugTask;
            _setpointTask = setpointTask;

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;

        typedef enum {
            STATUS_IDLE,
            STATUS_ARMED,
            STATUS_FLYING,
            STATUS_LANDING,
            STATUS_LOST_CONTACT

        } status_t;

        static void runCoreTask(void *arg)
        {
            ((CoreTask *)arg)->run();
        }

        FreeRtosTask _task;

        DebugTask * _debugTask;
        ImuTask * _imuTask;
        SetpointTask * _setpointTask;

        status_t status = STATUS_IDLE;

        void run()
        {
            for (uint32_t step=1; ; step++) {

                // Wait for IMU
                _imuTask->waitDataReady();

                // Get setpoint
                setpoint_t setpoint = {};
                _setpointTask->getSetpoint(setpoint);

                if (setpoint.timestamp > 0 &&
                        xTaskGetTickCount() - setpoint.timestamp > SETPOINT_TIMEOUT_TICKS) {
                    status = STATUS_LOST_CONTACT;
                }

                if (status == STATUS_LOST_CONTACT) {
                    // No way to recover from this
                    DebugTask::setMessage(_debugTask, "%05d: lost contact", step);
                }

                else switch (status) {

                    case STATUS_IDLE:
                        DebugTask::setMessage(_debugTask, "%05d: idle: arming=%d",
                                step, setpoint.arming);
                        break;

                    case STATUS_ARMED:
                        DebugTask::setMessage(_debugTask, "%05d: armed", step);
                        break;

                    case STATUS_FLYING:
                        DebugTask::setMessage(_debugTask, "%05d: flying", step);
                        break;

                    case STATUS_LANDING:
                        DebugTask::setMessage(_debugTask, "%05d: landing", step);
                        break;
                }
            }
        }

        // Device-dependent ---------------------------

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
