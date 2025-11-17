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
#include <__messages__.h>
#include <debugger.hpp>
#include <ekf.hpp>
#include <imu.hpp>
#include <msp/parser.hpp>
#include <task.hpp>
#include <timer.hpp>
#include <vehicles/diyquad.hpp>

class CoreTask {

    public:

        void begin(
                EKF * ekf,
                const uint8_t motorCount,
                const mixFun_t mixFun,
                Debugger * debugger=nullptr)
        {
            _ekf = ekf;
            _debugger = debugger;
            _motorCount = motorCount;
            _mixFun = mixFun;

            _imu.init();

            _task.init(runCoreTask, "core", this, TASK_PRIORITY);
        }

    private:

        static constexpr float FLYING_STATUS_FREQ = 25;
        static constexpr float CORE_FREQ = 1000;
        static constexpr float COMMS_FREQ = 100;

        static constexpr float LED_HEARTBEAT_FREQ = 1;
        static constexpr float LED_IMU_CALIBRATING_FREQ = 3;
        static constexpr uint32_t LED_PULSE_DURATION_MSEC = 50;

        static const uint8_t TASK_PRIORITY = 5;
        static const uint32_t COMMAND_TIMEOUT_MSEC = 100;
        static constexpr float STATE_PHITHETA_MAX = 30;
        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;
        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs
        static const auto CLOSED_LOOP_UPDATE_FREQ = 500;

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t EKF_PREDICTION_FREQ = 100;

        static constexpr float MAX_VELOCITY = 10; //meters per second

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

        Task _task;

        Debugger * _debugger;
        mixFun_t _mixFun;
        uint8_t _motorCount;
        EKF * _ekf;
        Imu _imu;

        void run()
        {
            status_t status = STATUS_IDLE;

            // Start with motor speeds at idle
            float motorvals[MAX_MOTOR_COUNT] = {};

            // Start with no axis demands
            demands_t demands = {};

            // Run device-dependent motor initialization
            motors_init();

            // Run device-dependent LED initialization
            led_init();

            comms_init();

            // Start serial debugging
            Serial.begin(115200);

            uint32_t nextPredictionMs = millis();

            const uint32_t msec_start = millis();

            bool isFlying = false;

            ClosedLoopControl closedLoopControl  = {};

            vehicleState_t vehicleState = {};

            command_t command = {};

            bool didResetEstimation = false;

            Timer flyingStatusTimer = {};

            while (true) {

                const uint32_t msec = millis() - msec_start;

                // Sync the core loop to the IMU
                const bool imuIsCalibrated = _imu.step(_ekf, msec);
                Task::wait(CORE_FREQ);

                // Set the LED based on current status
                runLed(imuIsCalibrated, status);

                // Run logging
                runLogger(vehicleState, closedLoopControl);

                // Get command
                runCommandParser(command);

                // Periodically update ekf with flying status
                if (flyingStatusTimer.ready(FLYING_STATUS_FREQ)) {
                    isFlying = isFlyingCheck(msec, motorvals);
                }

                // Run ekf to get vehicle state
                getStateEstimate(isFlying, vehicleState, nextPredictionMs,
                        didResetEstimation);

                // Check for lost contact
                if (command.timestamp > 0 &&
                        millis() - command.timestamp > COMMAND_TIMEOUT_MSEC) {
                    status = STATUS_LOST_CONTACT;
                }

                switch (status) {

                    case STATUS_IDLE:
                        if (command.armed && isSafeAngle(vehicleState.phi) &&
                                isSafeAngle(vehicleState.theta)) {
                            status = STATUS_ARMED;
                        }
                        runMotors(motorvals);
                        break;

                    case STATUS_ARMED:
                        checkDisarm(command, status, motorvals);
                        if (command.hovering) {
                            status = STATUS_HOVERING;
                        }
                        runMotors(motorvals);
                        break;

                    case STATUS_HOVERING:
                        runClosedLoopAndMixer(command, vehicleState,
                                status, closedLoopControl, demands, motorvals);
                        if (!command.hovering) {
                            status = STATUS_LANDING;
                        }
                        break;

                    case STATUS_LANDING:
                        runClosedLoopAndMixer(command, vehicleState,
                                status, closedLoopControl, demands, motorvals);
                        break;

                    case STATUS_LOST_CONTACT:
                        // No way to recover from this
                        break;
                }
            }
        }

        void getStateEstimate(
                const bool isFlying,
                vehicleState_t & state,
                uint32_t & nextPredictionMs,
                bool & didResetEstimation)
        {
            const uint32_t nowMs = millis();

            if (didResetEstimation) {
                _ekf->init(nowMs);
               didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (nowMs >= nextPredictionMs) {
                _ekf->predict(nowMs, isFlying); 
                nextPredictionMs = nowMs + (1000 / EKF_PREDICTION_FREQ);
            }

            axis3_t dpos = {};
            axis4_t quat = {};

            _ekf->getStateEstimate(nowMs, state.z, dpos, quat);

            if (!velInBounds(dpos.x) || !velInBounds(dpos.y) ||
                    !velInBounds(dpos.z)) {
                didResetEstimation = true;
            }

            state.dx = dpos.x;
            state.dy = -dpos.y; // negate for rightward positive
            state.dz = dpos.z;

            axis3_t angles = {};
            Num::quat2euler(quat, angles);

            state.phi = angles.x;
            state.theta = angles.y;
            state.psi = -angles.z; // negate for nose-right positive

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

        void runClosedLoopAndMixer(
                const command_t &command,
                const vehicleState_t & state,
                status_t & status,
                ClosedLoopControl & control,
                demands_t & demands,
                float *motorvals)
        {
            static Timer _timer;

            if (_timer.ready(CLOSED_LOOP_UPDATE_FREQ)) {

                control.run(1.f / CLOSED_LOOP_UPDATE_FREQ,
                        command.hovering, state, command.demands,
                        demands);

                runMixer(_mixFun, demands, motorvals);

                runMotors(motorvals);

                checkDisarm(command, status, motorvals);
            }
        }

        void runMotors(float * motorvals)
        {
            for (uint8_t k=0; k<_motorCount; ++k) {
                motors_setSpeed(k, motorvals[k]);
            }
            motors_run();
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

        void checkDisarm(const command_t command, status_t &status,
                float * motorvals)
        {
            if (!command.armed) {
                status = STATUS_IDLE;
                memset(motorvals, 0, _motorCount * sizeof(motorvals));
            }
        }

        //
        // We say we are flying if one or more motors are running over the idle
        // thrust.
        //
        bool isFlyingCheck(const uint32_t step, const float * motorvals)
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

        void runLogger(const vehicleState_t & state,
                ClosedLoopControl & control)
        {
            static Timer _timer;

            if (_timer.ready(COMMS_FREQ)) {
                sendVehicleState(state);
                sendClosedLoopControlMessage(control);
            }
        }

        void sendVehicleState(const vehicleState_t & state)
        {
            MspSerializer serializer = {};

            const float statevals[10] = {
                state.dx,
                state.dy,
                state.z,
                state.dz,
                state.phi,
                state.dphi,
                state.theta,
                state.dtheta,
                state.psi,
                state.dpsi
            };

            serializer.serializeFloats(MSP_STATE, statevals, 10);

            sendPayload(serializer);
        }

        void sendClosedLoopControlMessage(ClosedLoopControl & control)
        {
            MspSerializer serializer = {};

            control.serializeMessage(serializer);

            sendPayload(serializer);
        }

        void sendPayload(const MspSerializer & serializer) {
            for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                comms_write_byte(serializer.payload[k]);
            }
        }

        void runLed(const bool imuIsCalibrated, const status_t status)
        {
            const uint32_t msec_curr = millis();

            if (!imuIsCalibrated) {
                blinkLed(msec_curr, LED_IMU_CALIBRATING_FREQ);
            }

            else if (status == STATUS_ARMED ||
                    status == STATUS_HOVERING || 
                    status == STATUS_LANDING) { 
                led_set(true);
            }
            else {
                blinkLed(msec_curr, LED_HEARTBEAT_FREQ);
            }
        }

        void blinkLed(const uint32_t msec_curr, const float freq)
        {
            static bool _pulsing;
            static uint32_t _pulse_start;
            static Timer _timer;

            if (_timer.ready(freq)) {
                led_set(true);
                _pulsing = true;
                _pulse_start = msec_curr;
            }

            else if (_pulsing) {
                if (millis() - _pulse_start > LED_PULSE_DURATION_MSEC) {
                    led_set(false);
                    _pulsing = false;
                }
            }
        }

        void runCommandParser(command_t & command)
        {
            static Timer _timer;

            if (_timer.ready(COMMS_FREQ)) {

                static MspParser commandParser;

                uint8_t byte = 0;

                while (comms_read_byte(&byte)) {

                    switch (commandParser.parse(byte)) {

                        case MSP_SET_ARMING:
                            command.armed = !command.armed;
                            command.timestamp = millis();
                            break;

                        case MSP_SET_IDLE:
                            command.hovering = false;
                            command.timestamp = millis();
                            break;

                        case MSP_SET_SETPOINT:
                            command.hovering = true;
                            command.demands.pitch = commandParser.getFloat(0);
                            command.demands.roll = commandParser.getFloat(1);
                            command.demands.yaw = commandParser.getFloat(2);
                            command.demands.thrust = commandParser.getFloat(3);
                            command.timestamp = millis();
                            break;

                        default:
                            break;
                    }
                }
            }
        }

        static bool isSafeAngle(float angle)
        {
            return fabs(angle) < STATE_PHITHETA_MAX;
        }

        // Device-dependent ---------------------------

        void comms_init();

        bool comms_read_byte(uint8_t * byte);

        void comms_write_byte(const uint8_t byte);

        void led_init();

        void led_set(const bool on);

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
