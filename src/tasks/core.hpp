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
            _closedLoopControl = closedLoopControl;
            _estimatorTask = estimatorTask;
            _imuTask = imuTask;
            _ledTask = ledTask;
            _setpointTask = setpointTask;
            _debugTask = debugTask;
            _motorCount = motorCount;
            _mixFun = mixFun;

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static constexpr float LANDING_ALTITUDE_M = 0.03;
        static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;
        static constexpr float MAX_SAFE_ANGLE = 30;
        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;
        static const Clock::rate_t FLYING_STATUS_CLOCK_RATE = Clock::RATE_25_HZ;
        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs

        static const auto CLOSED_LOOP_UPDATE_RATE = Clock::RATE_500_HZ; // Needed ?

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

        ClosedLoopControl * _closedLoopControl;
        mixFun_t _mixFun;
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

            // Start with no axis demands
            demands_t demands = {};

            // Run device-dependent motor initialization
            motors_init();

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

                switch (status) {

                    case STATUS_IDLE:
                        reportStatus(step, "idle", motorvals);
                        if (setpoint.armed && isSafeAngle(_vehicleState.phi) &&
                                isSafeAngle(_vehicleState.theta)) {
                            _ledTask->setArmed(true);
                            status = STATUS_ARMED;
                        }
                        break;

                    case STATUS_ARMED:
                        reportStatus(step, "armed", motorvals);
                        checkDisarm(setpoint, status, motorvals);
                        if (setpoint.hovering) {
                            status = STATUS_HOVERING;
                        }
                        break;

                    case STATUS_HOVERING:
                        reportStatus(step, "hovering", motorvals);
                        runClosedLoopAndMixer(step, setpoint,
                                demands, motorvals);
                        checkDisarm(setpoint, status, motorvals);
                        if (!setpoint.hovering) {
                            status = STATUS_LANDING;
                        }
                        break;

                    case STATUS_LANDING:
                        reportStatus(step, "landing", motorvals);
                        checkDisarm(setpoint, status, motorvals);
                        break;

                    case STATUS_LOST_CONTACT:
                        // No way to recover from this
                        DebugTask::setMessage(_debugTask, "%05d: lost contact", step);
                        break;
                }
            }
        }

        void runClosedLoopAndMixer(
                const uint32_t step, setpoint_t &setpoint,
                demands_t & demands, float *motorvals)
        {
            if (Clock::rateDoExecute(CLOSED_LOOP_UPDATE_RATE, step)) {
                runClosedLoopControl(setpoint, demands);

                // Run closedLoopDemands through mixer to get motor speeds
                runMixer(_mixFun, demands, motorvals);
            }
        }

        void runClosedLoopControl(
                setpoint_t & setpoint, demands_t & closedLoopDemands)
        {
            if (setpoint.hovering) {

                setpoint.demands.thrust = Num::rescale(
                        setpoint.demands.thrust, 0.2, 2.0, -1, +1);

                setpoint.demands.thrust = Num::rescale(
                        setpoint.demands.thrust, -1, +1, 0.2, 2.0);
            }

            _closedLoopControl->run(
                    1.f / CLOSED_LOOP_UPDATE_RATE,
                    setpoint.hovering,
                    _vehicleState,
                    setpoint.demands,
                    LANDING_ALTITUDE_M,
                    closedLoopDemands);
        }

        void runMixer(const mixFun_t mixFun, const demands_t & demands,
                float * motorvals)
        {
            float uncapped[MAX_MOTOR_COUNT] = {};
            mixFun(demands, uncapped);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<_motorCount; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > THRUST_MAX) {
                reduction = highestThrustFound - THRUST_MAX;
            }

            for (uint8_t k = 0; k < _motorCount; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = thrustCappedUpper < 0 ? 0 : thrustCappedUpper / 65536;
            }
        }

        void reportStatus(const uint32_t step, const char * status,
                const float * motorvals)
        {
            DebugTask::setMessage(_debugTask,
                    "%05d: %-8s    m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f", 
                    step, status,
                    motorvals[0], motorvals[1], motorvals[2], motorvals[3]);
        }

        void checkDisarm(const setpoint_t setpoint, status_t &status,
                float * motorvals)
        {
            if (!setpoint.armed) {
                status = STATUS_IDLE;
                memset(motorvals, 0, _motorCount * sizeof(motorvals));
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
