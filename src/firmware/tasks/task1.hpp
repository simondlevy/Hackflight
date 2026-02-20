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


#include <firmware/estimators/ekf.hpp>
#include <firmware/imu.hpp>
#include <firmware/led.hpp>
#include <firmware/logger.hpp>
#include <mixers/crazyflie.hpp>
#include <firmware/rc.hpp>
#include <firmware/task.hpp>
#include <pidcontrol.hpp>
#include <vehicles/diyquad.hpp>

namespace hf {

    class Task1 {

        public:

            void begin(EKF * ekf)
            {
                _ekf = ekf;

                _imu.init();

                _task.init(runTask1, "task1", this, 5);
            }

        private:

            static bool rateDoExecute(const uint32_t rate, const uint32_t tick)
            {
                return (tick % (FREQ_MAIN_LOOP / rate)) == 0;
            }

            static const uint32_t SETPOINT_TIMEOUT_TICKS = 1000;
            static constexpr float MAX_SAFE_ANGLE = 30;
            static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;

            static const uint32_t FREQ_MAIN_LOOP = 1000;
            static const uint32_t FREQ_PID_UPDATE = 500;
            static const uint32_t FREQ_EKF_PREDICTION = 100;
            static const uint32_t FREQ_FLYING_MODE_CHECK = 25;

            static constexpr float TILT_ANGLE_FLIPPED_MIN = 75;


            static constexpr float MAX_VELOCITY = 10; //meters per second

            static void runTask1(void *arg)
            {
                ((Task1 *)arg)->run();
            }

            IMU _imu;

            vehicleState_t _vehicleState;
            Led _led;
            PidControl _pidControl;

            FreeRtosTask _task;

            EKF * _ekf;

            void run()
            {
                mode_e mode = MODE_IDLE;

                // Start with motor speeds at idle
                float motorvals[MAX_MOTOR_COUNT] = {};

                // Start with no axis setpoint
                setpoint_t setpoint = {};

                // Run device-dependent motor initialization
                motors_init();

                _led.init();

                RC::message_t message = {};

                const uint32_t msec_start = millis();

                bool isFlying = false;

                bool didResetEstimation = false;

                for (uint32_t step=1; ; step++) {

                    // Yield
                    vTaskDelay(1000/FREQ_MAIN_LOOP);

                    const uint32_t msec = millis() - msec_start;

                    // Sync the core loop to the IMU
                    const bool imuIsCalibrated = _imu.step(_ekf, msec);

                    // Get message from remote control
                    RC::getSetpoint(xTaskGetTickCount(), message);

                    // Periodically update estimator with flying mode
                    if (rateDoExecute(FREQ_FLYING_MODE_CHECK, step)) {
                        isFlying = isFlyingCheck(msec, motorvals);
                    }

                    // Run ekf to get vehicle state
                    getStateEstimate(isFlying, _vehicleState, didResetEstimation);

                    // Check for lost contact
                    if (message.timestamp > 0 &&
                            xTaskGetTickCount() - message.timestamp >
                            SETPOINT_TIMEOUT_TICKS) {
                        mode = MODE_PANIC;
                    }

                    // Check for flipped over
                    if (isFlippedAngle(_vehicleState.theta) ||
                            isFlippedAngle(_vehicleState.phi)) {
                        mode = MODE_PANIC;
                    }

                    _led.run(millis(), imuIsCalibrated, mode);

                    Logger::run(millis(), _vehicleState);

                    switch (mode) {

                        case MODE_IDLE:
                            if (message.armed && isSafeAngle(_vehicleState.phi)
                                    && isSafeAngle(_vehicleState.theta)) {
                                mode = MODE_ARMED;
                            }
                            runMotors(motorvals);
                            break;

                        case MODE_ARMED:
                            checkDisarm(message, mode, motorvals);
                            if (message.hovering) {
                                mode = MODE_HOVERING;
                            }
                            runMotors(motorvals);
                            break;

                        case MODE_HOVERING:
                            runPidAndMixer(step, message,
                                    setpoint, motorvals);
                            checkDisarm(message, mode, motorvals);
                            if (!message.hovering) {
                                mode = MODE_LANDING;
                            }
                            break;

                        case MODE_LANDING:
                            runPidAndMixer(step, message,
                                    setpoint, motorvals);
                            checkDisarm(message, mode, motorvals);
                            break;

                        case MODE_PANIC:
                            // No way to recover from this
                            stopMotors();
                            break;
                    }
                }
            }

            static bool isFlippedAngle(float angle)
            {
                return fabs(angle) > TILT_ANGLE_FLIPPED_MIN;
            }

            void getStateEstimate(
                    const bool isFlying,
                    vehicleState_t & state,
                    bool & didResetEstimation)
            {
                static Timer _timer;

                const uint32_t nowMs = millis();

                if (didResetEstimation) {
                    _ekf->init(nowMs);
                    didResetEstimation = false;
                }

                // Run the system dynamics to predict the state forward.
                if (_timer.ready(FREQ_EKF_PREDICTION)) {
                    _ekf->predict(nowMs, isFlying); 
                }

                // Get state estimate from EKF
                _ekf->getStateEstimate(nowMs, state);

                if (!velInBounds(state.dx) || !velInBounds(state.dy) ||
                        !velInBounds(state.dz)) {
                    didResetEstimation = true;
                }

                // Get angular velocities directly from gyro
                axis3_t gyroData = {};
                _imu.getGyroData(gyroData);
                state.dphi   = gyroData.x;
                state.dtheta = gyroData.y;
                state.dpsi   = -gyroData.z; // negate for nose-right positive
            }

            static bool velInBounds(const float vel)
            {
                return fabs(vel) < MAX_VELOCITY;
            }

            //
            // We say we are flying if one or more motors are running over the idle
            // thrust.
            //
            bool isFlyingCheck(const uint32_t step, const float * motorvals)
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
                    latestThrustTick = step;
                }

                bool result = false;
                if (0 != latestThrustTick) {
                    if ((step - latestThrustTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
                        result = true;
                    }
                }

                return result;
            }        


            void runPidAndMixer(
                    const uint32_t step, const RC::message_t &message,
                    setpoint_t & setpoint, float *motorvals)
            {
                if (rateDoExecute(FREQ_PID_UPDATE, step)) {

                    _pidControl.run(
                            1.f / FREQ_PID_UPDATE,
                            message.hovering,
                            _vehicleState,
                            message.setpoint,
                            setpoint);

                    runMixer(setpoint, motorvals);

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

            void stopMotors()
            {
                for (uint8_t k=0; k<Mixer::rotorCount; ++k) {
                    motors_setSpeed(k, 0);
                }
                motors_run();
            }

            void runMixer(const setpoint_t & setpoint, float * motorvals)
            {
                float uncapped[MAX_MOTOR_COUNT] = {};
                Mixer::mix(setpoint, uncapped);

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

            void checkDisarm(const RC::message_t message, mode_e &mode,
                    float * motorvals)
            {
                if (!message.armed) {
                    mode = MODE_IDLE;
                    memset(motorvals, 0, Mixer::rotorCount * sizeof(motorvals));
                }
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

}
