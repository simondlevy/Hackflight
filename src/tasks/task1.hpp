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


#include <led.hpp>
#include <logger.hpp>
#include <newekf.hpp>
#include <mixers/crazyflie.hpp>
#include <pidcontrol.hpp>
#include <rc.hpp>
#include <task.hpp>
#include <tasks/estimator.hpp>
#include <tasks/imu.hpp>
#include <vehicles/diyquad.hpp>

#include <newekf.hpp>
#include <newimu.hpp>

class Task1 {

    public:

        void begin(
                EstimatorTask * estimatorTask,
                ImuTask * imuTask,
                NewEKF * ekf)
        {
            _estimatorTask = estimatorTask;
            _imuTask = imuTask;

            _ekf = ekf;

            _task.init(runTask1, "task1", this, 5);
        }

    private:

        static constexpr float LANDING_ALTITUDE_M = 0.03;
        static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;
        static constexpr float MAX_SAFE_ANGLE = 30;
        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;
        static const Clock::rate_t FLYING_MODE_CLOCK_RATE =
            Clock::RATE_25_HZ;
        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs

        static const auto CLOSED_LOOP_UPDATE_RATE =
            Clock::RATE_500_HZ; // Needed ?

        static void runTask1(void *arg)
        {
            ((Task1 *)arg)->run();
        }

        vehicleState_t _vehicleState;
        Led _led;
        PidControl _pidControl;

        FreeRtosTask _task;
        EstimatorTask * _estimatorTask;
        ImuTask * _imuTask;

        NewEKF * _ekf;

        void run()
        {
            mode_e status = MODE_IDLE;

            // Start with motor speeds at idle
            float motorvals[MAX_MOTOR_COUNT] = {};

            // Start with no axis demands
            demands_t demands = {};

            // Run device-dependent motor initialization
            motors_init();

            _led.init();

            setpoint_t setpoint = {};

            for (uint32_t step=1; ; step++) {

                // Yield
                vTaskDelay(1);

                // Wait for IMU
                _imuTask->waitDataReady();

                // Get setpoint from remote control
                RC::getSetpoint(xTaskGetTickCount(), setpoint);

                // Periodically update estimator with flying status
                if (Clock::rateDoExecute(FLYING_MODE_CLOCK_RATE, step)) {
                    _estimatorTask->setFlyingStatus(
                            isFlyingCheck(xTaskGetTickCount(), motorvals));
                }

                // Get vehicle state from estimator
                _estimatorTask->getVehicleState(&_vehicleState);

                // Check for lost contact
                if (setpoint.timestamp > 0 &&
                        xTaskGetTickCount() - setpoint.timestamp >
                        SETPOINT_TIMEOUT_TICKS) {
                    status = MODE_PANIC;
                }

                _led.run(millis(), _imuTask->imuIsCalibrated(), status);

                Logger::run(millis(), _estimatorTask);

                switch (status) {

                    case MODE_IDLE:
                        if (setpoint.armed && isSafeAngle(_vehicleState.phi)
                                && isSafeAngle(_vehicleState.theta)) {
                            status = MODE_ARMED;
                        }
                        runMotors(motorvals);
                        break;

                    case MODE_ARMED:
                        checkDisarm(setpoint, status, motorvals);
                        if (setpoint.hovering) {
                            status = MODE_HOVERING;
                        }
                        runMotors(motorvals);
                        break;

                    case MODE_HOVERING:
                        runClosedLoopAndMixer(step, setpoint,
                                demands, motorvals);
                        checkDisarm(setpoint, status, motorvals);
                        if (!setpoint.hovering) {
                            status = MODE_LANDING;
                        }
                        break;

                    case MODE_LANDING:
                        runClosedLoopAndMixer(step, setpoint,
                                demands, motorvals);
                        checkDisarm(setpoint, status, motorvals);
                        break;

                    case MODE_PANIC:
                        // No way to recover from this
                        break;
                }
            }
        }

        void runClosedLoopAndMixer(
                const uint32_t step, const setpoint_t &setpoint,
                demands_t & demands, float *motorvals)
        {
            if (Clock::rateDoExecute(CLOSED_LOOP_UPDATE_RATE, step)) {

                _pidControl.run(
                        1.f / CLOSED_LOOP_UPDATE_RATE,
                        setpoint.hovering,
                        _vehicleState,
                        setpoint.demands,
                        LANDING_ALTITUDE_M,
                        demands);

                runMixer(demands, motorvals);

                runMotors(motorvals);
            }
        }

        void runMotors(float * motorvals)
        {
            for (uint8_t k=0; k<Mixer::rotorCount; ++k) {
                motors_setSpeed(k, motorvals[k]);
            }
            motors_run();
        }

        void runMixer(const demands_t & demands, float * motorvals)
        {
            float uncapped[MAX_MOTOR_COUNT] = {};
            Mixer::mix(demands, uncapped);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<Mixer::rotorCount; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > THRUST_MAX) {
                reduction = highestThrustFound - THRUST_MAX;
            }

            for (uint8_t k = 0; k < Mixer::rotorCount; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] =
                    thrustCappedUpper < 0 ? 0 : thrustCappedUpper / 65536;
            }
        }

        void checkDisarm(const setpoint_t setpoint, mode_e &status,
                float * motorvals)
        {
            if (!setpoint.armed) {
                status = MODE_IDLE;
                memset(motorvals, 0, Mixer::rotorCount * sizeof(motorvals));
            }
        }

        //
        // We say we are flying if one or more motors are running over the
        // idle thrust.
        //
        bool isFlyingCheck(const uint32_t tick, const float * motorvals)
        {
            auto isThrustOverIdle = false;

            for (int i = 0; i < Mixer::rotorCount; ++i) {
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
                if ((tick - latestThrustTick) <
                        IS_FLYING_HYSTERESIS_THRESHOLD) {
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
