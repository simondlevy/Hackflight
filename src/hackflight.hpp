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

#include <Wire.h>
#include <SPI.h>

#include <__control__.hpp>
#include <__messages__.h>

#include <debugger.hpp>
#include <ekf.hpp>
#include <imu.hpp>
#include <opticalflow.hpp>
#include <msp/parser.hpp>
#include <timer.hpp>
#include <vehicles/diyquad.hpp>
#include <zranger.hpp>

class Hackflight {

    public:

        void init()
        {
            _ekf.init(millis());

            _imu.init();

            _zranger.init(wire_device());

            _opticalflow.init(spi_device(), spi_cs_pin());

            motors_init();

            pinMode(led_pin(), OUTPUT);

            comms_init();

            Serial.begin(115200);

            _msec_start = millis();
        }

        void loop1(const uint8_t motorCount, const mixFun_t mixFun)
        {
            static status_t _status;
            static float _motorvals[MAX_MOTOR_COUNT];
            static demands_t _demands;
            static bool _isFlying;
            static ClosedLoopControl _closedLoopControl;
            static vehicleState_t _vehicleState;
            static command_t _command;
            static bool _didResetEstimation;
            static Timer _flyingStatusTimer;

            const uint32_t msec = millis() - _msec_start;

            // Sync the core loop to the IMU
            const bool imuIsCalibrated = _imu.step(&_ekf, msec);

            // Set the LED based on current status
            runLed(imuIsCalibrated, _status);

            // Run logging
            runLogger(_vehicleState, _closedLoopControl);

            // Get command
            runCommandParser(_command);

            // Periodically update ekf with flying status
            if (_flyingStatusTimer.ready(FLYING_STATUS_FREQ)) {
                _isFlying = isFlyingCheck(msec, motorCount, _motorvals);
            }

            // Run ekf to get vehicle state
            getStateEstimate(_isFlying, _vehicleState, _didResetEstimation);

            // Check for lost contact
            if (_command.timestamp > 0 &&
                    millis() - _command.timestamp > COMMAND_TIMEOUT_MSEC) {
                _status = STATUS_LOST_CONTACT;
            }

            switch (_status) {

                case STATUS_IDLE:
                    if (_command.armed && isSafeAngle(_vehicleState.phi) &&
                            isSafeAngle(_vehicleState.theta)) {
                        _status = STATUS_ARMED;
                    }
                    runMotors(motorCount, _motorvals);
                    break;

                case STATUS_ARMED:
                    checkDisarm(_command, motorCount, _status, _motorvals);
                    if (_command.hovering) {
                        _status = STATUS_HOVERING;
                    }
                    runMotors(motorCount, _motorvals);
                    break;

                case STATUS_HOVERING:
                    runClosedLoopAndMixer(_command, _vehicleState, motorCount,
                            mixFun, _status, _closedLoopControl, _demands,
                            _motorvals);
                    if (!_command.hovering) {
                        _status = STATUS_LANDING;
                    }
                    break;

                case STATUS_LANDING:
                    runClosedLoopAndMixer(_command, _vehicleState, motorCount,
                            mixFun, _status, _closedLoopControl, _demands,
                            _motorvals);
                    break;

                case STATUS_LOST_CONTACT:
                    // No way to recover from this
                    break;
            }
        }

        void loop2()
        {
            _zranger.step(&_ekf);
            _opticalflow.step(&_ekf);
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

        Imu _imu;
        OpticalFlow _opticalflow;
        ZRanger _zranger;

        EKF _ekf;
        Debugger _debugger;
        uint32_t _msec_start;

        void getStateEstimate(
                const bool isFlying,
                vehicleState_t & state,
                bool & didResetEstimation)
        {
            static Timer _timer;

            const uint32_t nowMs = millis();

            if (didResetEstimation) {
                _ekf.init(nowMs);
                didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (_timer.ready(EKF_PREDICTION_FREQ)) {
                _ekf.predict(nowMs, isFlying); 
            }

            axis3_t dpos = {};
            axis4_t quat = {};

            _ekf.getStateEstimate(nowMs, state.z, dpos, quat);

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
                const uint8_t motorCount,
                const mixFun_t mixFun,
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

                runMixer(motorCount, mixFun, demands, motorvals);

                runMotors(motorCount, motorvals);

                checkDisarm(command, motorCount, status, motorvals);
            }
        }

        void runMotors(const uint8_t motorCount, float * motorvals)
        {
            for (uint8_t k=0; k<motorCount; ++k) {
                motors_setSpeed(k, motorvals[k]);
            }
            motors_run();
        }

        void runMixer(
                const uint8_t motorCount,
                const mixFun_t mixFun,
                const demands_t & demands,
                float * motorvals)
        {
            float uncapped[MAX_MOTOR_COUNT] = {};
            mixFun(demands, uncapped);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<motorCount; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > THRUST_MAX) {
                reduction = highestThrustFound - THRUST_MAX;
            }

            for (uint8_t k = 0; k < motorCount; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = thrustCappedUpper < 0 ? 0 : thrustCappedUpper / 65536;
            }
        }

        void checkDisarm(
                const command_t command,
                const uint8_t motorCount,
                status_t &status,
                float * motorvals)
        {
            if (!command.armed) {
                status = STATUS_IDLE;
                for (uint8_t k=0; k<motorCount; ++k) {
                    motorvals[k] = 0;
                }
            }
        }

        //
        // We say we are flying if one or more motors are running over the idle
        // thrust.
        //
        bool isFlyingCheck(
                const uint32_t step,
                const uint8_t motorCount,
                const float * motorvals)
        {
            auto isThrustOverIdle = false;

            for (int i = 0; i < motorCount; ++i) {
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
                digitalWrite(led_pin(), !led_inverted());
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
                digitalWrite(led_pin(), !led_inverted());
                _pulsing = true;
                _pulse_start = msec_curr;
            }

            else if (_pulsing) {
                if (millis() - _pulse_start > LED_PULSE_DURATION_MSEC) {
                    digitalWrite(led_pin(), led_inverted());
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

        TwoWire * wire_device();

        SPIClass * spi_device();

        const uint8_t spi_cs_pin();

        void comms_init();

        bool comms_read_byte(uint8_t * byte);

        void comms_write_byte(const uint8_t byte);

        const uint8_t led_pin();

        const bool led_inverted();

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
