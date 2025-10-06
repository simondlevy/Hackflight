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
#include <tasks/imu.hpp>
#include <tasks/led.hpp>
#include <tasks/setpoint.hpp>

class CoreTask {

    public:

        void begin(
                ClosedLoopControl * closedLoopControl,
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                LedTask * ledTask,
                SetpointTask * setpointTask,
                const uint8_t motorCount,
                const mixFun_t mixFun,
                DebugTask * debugTask=nullptr)
        {
            _imuTask = imuTask;
            _debugTask = debugTask;
            _estimatorTask = estimatorTask;
            _ledTask = ledTask;
            _setpointTask = setpointTask;

            _motorCount = motorCount;

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;

        static constexpr float MAX_SAFE_ANGLE = 30;

        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;

        static const Clock::rate_t FLYING_STATUS_CLOCK_RATE = Clock::RATE_25_HZ;

        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs

        typedef enum {
            STATUS_IDLE,
            STATUS_ARMED,
            STATUS_HOVERING,
            STATUS_LANDING,
            STATUS_LOST_CONTACT

        } status_t;

        static void runCoreTask(void *arg)
        {
            ((CoreTask *)arg)->run();
        }

        FreeRtosTask _task;

        DebugTask * _debugTask;
        EstimatorTask * _estimatorTask;
        ImuTask * _imuTask;
        LedTask * _ledTask;
        SetpointTask * _setpointTask;

        vehicleState_t _vehicleState;

        uint8_t _motorCount;

        void run()
        {
            status_t status = STATUS_IDLE;

            // Start with motor speeds at idle
            float motorvals[MAX_MOTOR_COUNT] = {};

            for (uint32_t step=1; ; step++) {

                // Wait for IMU
                _imuTask->waitDataReady();

                // Get setpoint
                setpoint_t setpoint = {};
                _setpointTask->getSetpoint(setpoint);

                // Periodically update estimator with flying status
                if (Clock::rateDoExecute(FLYING_STATUS_CLOCK_RATE, step)) {
                    _estimatorTask->setFlyingStatus(
                            isFlyingCheck(xTaskGetTickCount(), motorvals));
                }

                // Get vehicle state from estimator
                _estimatorTask->getVehicleState(&_vehicleState);

                // Check for lost contact
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
                        reportStatus(step, "idle", motorvals);
                        if (setpoint.arming && isSafeAngle(_vehicleState.phi) &&
                                isSafeAngle(_vehicleState.theta)) {
                            _ledTask->setArmed(true);
                            status = STATUS_ARMED;
                        }
                        break;

                    case STATUS_ARMED:
                        reportStatus(step, "armed", motorvals);
                        checkDisarm(setpoint, status);
                        if (setpoint.hovering) {
                            status = STATUS_HOVERING;
                        }
                        break;

                    case STATUS_HOVERING:
                        reportStatus(step, "hovering", motorvals);
                        checkDisarm(setpoint, status);
                        if (!setpoint.hovering) {
                            status = STATUS_LANDING;
                        }
                        break;

                    case STATUS_LANDING:
                        reportStatus(step, "landing", motorvals);
                        checkDisarm(setpoint, status);
                        break;
                }
            }
        }

        void reportStatus(const uint32_t step, const char * status, const float * motorvals)
        {
            DebugTask::setMessage(_debugTask, "%05d: %s", step, status);
        }

        void checkDisarm(const setpoint_t setpoint, status_t &status)
        {
            if (!setpoint.arming) {
                status = STATUS_IDLE;
                _ledTask->setArmed(false);
            }
        }

        //
        // We say we are flying if one or more motors are running over the idle
        // thrust.
        //
        bool isFlyingCheck(const uint32_t tick, const float * motorvals)
        {
            auto isThrustOverIdle = false;

            for (int i = 0; i < _motorCount; ++i) {
                if (motorvals[i] > 0) {
                    isThrustOverIdle = true;
                    break;
                }
            }

            static uint32_t latestThrustTick;

            if (isThrustOverIdle) {
                latestThrustTick = tick;
            }

            bool result = false;
            if (0 != latestThrustTick) {
                if ((tick - latestThrustTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
                    result = true;
                }
            }

            return result;
        }        

        static bool isSafeAngle(float angle)
        {
            return fabs(angle) < MAX_SAFE_ANGLE;
        }

        // Device-dependent ---------------------------

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
