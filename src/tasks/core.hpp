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

            _imu.begin(ekf);

            _task.init(runCoreTask, "core", this, 5);
        }

    private:

        static const uint32_t COMMAND_TIMEOUT_TICKS = 1000;
        static constexpr float STATE_PHITHETA_MAX = 30;
        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;
        static const Timer::rate_t FLYING_STATUS_FREQ = Timer::FREQ_25_HZ;
        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs
        static const auto CLOSED_LOOP_UPDATE_FREQ = Timer::FREQ_500_HZ;

        static constexpr float LED_HEARTBEAT_FREQ = 1;
        static constexpr float LED_IMU_CALIBRATING_FREQ = 3;
        static constexpr uint32_t LED_PULSE_DURATION_MSEC = 50;

        static constexpr float COMMS_FREQ = 100;

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

        ClosedLoopControl _closedLoopControl;
        mixFun_t _mixFun;
        FreeRtosTask _task;
        Debugger * _debugger;
        EKF * _ekf;
        Imu _imu;
        vehicleState_t _vehicleState;

        bool _didResetEstimation;

        uint8_t _motorCount;

        Timer _ledTimer;

        Timer _loggingTimer;

        Timer _commandTimer;

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

            bool isFlying = false;

            for (uint32_t step=1; ; step++) {

                // Sync the core loop to the IMU
                _imu.step(xTaskGetTickCount());
                vTaskDelay(1000/Timer::CORE_FREQ);

                // Set the LED based on current status
                runLed(status);

                // Run logging
                runLogger();

                // Get command
                static command_t _command;
                runCommandParser(_command);

                // Periodically update ekf with flying status
                if (Timer::rateDoExecute(FLYING_STATUS_FREQ, step)) {
                    isFlying = isFlyingCheck(xTaskGetTickCount(), motorvals);
                }

                // Run ekf to get vehicle state
                nextPredictionMs = runEkf(isFlying, nextPredictionMs);

                // Check for lost contact
                if (_command.timestamp > 0 &&
                        xTaskGetTickCount() - _command.timestamp >
                        COMMAND_TIMEOUT_TICKS) {
                    status = STATUS_LOST_CONTACT;
                }

                switch (status) {

                    case STATUS_IDLE:
                        reportStatus(step, "idle", motorvals);
                        if (_command.armed && isSafeAngle(_vehicleState.phi) &&
                                isSafeAngle(_vehicleState.theta)) {
                            status = STATUS_ARMED;
                        }
                        runMotors(motorvals);
                        break;

                    case STATUS_ARMED:
                        reportStatus(step, "armed", motorvals);
                        checkDisarm(_command, status, motorvals);
                        if (_command.hovering) {
                            status = STATUS_HOVERING;
                        }
                        runMotors(motorvals);
                        break;

                    case STATUS_HOVERING:
                        reportStatus(step, "hovering", motorvals);
                        runClosedLoopAndMixer(step, _command,
                                demands, motorvals);
                        checkDisarm(_command, status, motorvals);
                        if (!_command.hovering) {
                            status = STATUS_LANDING;
                        }
                        break;

                    case STATUS_LANDING:
                        reportStatus(step, "landing", motorvals);
                        runClosedLoopAndMixer(step, _command,
                                demands, motorvals);
                        checkDisarm(_command, status, motorvals);
                        break;

                    case STATUS_LOST_CONTACT:
                        // No way to recover from this
                        Debugger::setMessage(_debugger, "%05d: lost contact", step);
                        break;
                }
            }
        }

        uint32_t runEkf(const bool isFlying, uint32_t nextPredictionMs)
        {
            const uint32_t nowMs = millis();

            if (_didResetEstimation) {
                _ekf->init(nowMs);
               _didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (nowMs >= nextPredictionMs) {
                _ekf->predict(nowMs, isFlying); 
                nextPredictionMs = nowMs + (1000 / EKF_PREDICTION_FREQ);
            }

            axis3_t dpos = {};
            axis4_t quat = {};
            axis3_t dangle = {};

            _ekf->getStateEstimate(nowMs, _vehicleState.z, dpos, dangle, quat);

            if (!velInBounds(dpos.x) || !velInBounds(dpos.y) ||
                    !velInBounds(dpos.z)) {
                _didResetEstimation = true;
            }

            _vehicleState.dx = dpos.x;
            _vehicleState.dy = -dpos.y; // negate for rightward positive
            _vehicleState.dz = dpos.z;

            axis3_t angles = {};
            Num::quat2euler(quat, angles);

            _vehicleState.phi = angles.x;
            _vehicleState.theta = angles.y;
            _vehicleState.psi = -angles.z; // negate for nose-right positive
            
            _vehicleState.dphi   = dangle.x;
            _vehicleState.dtheta = dangle.y;
            _vehicleState.dpsi   = -dangle.z; // negate for nose-right positive

            return nextPredictionMs;
        }

        static bool velInBounds(const float vel)
        {
            return fabs(vel) < MAX_VELOCITY;
        }

 
        void runClosedLoopAndMixer(
                const uint32_t step, const command_t &command,
                demands_t & demands, float *motorvals)
        {
            if (Timer::rateDoExecute(CLOSED_LOOP_UPDATE_FREQ, step)) {

                _closedLoopControl.run(step, 1.f / CLOSED_LOOP_UPDATE_FREQ,
                        command.hovering, _vehicleState, command.demands,
                        demands);

                runMixer(_mixFun, demands, motorvals);

                runMotors(motorvals);
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

        void reportStatus(const uint32_t step, const char * status,
                const float * motorvals)
        {
            Debugger::setMessage(_debugger,
                    "%05d: %-8s    m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f", 
                    step, status,
                    motorvals[0], motorvals[1], motorvals[2], motorvals[3]);
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

        void runLogger()
        {
            if (_loggingTimer.ready(COMMS_FREQ)) {
                sendVehicleState();
                sendClosedLoopControlMessage();
            }
        }

        void sendVehicleState()
        {
            MspSerializer serializer = {};

            const float statevals[10] = {
                _vehicleState.dx,
                _vehicleState.dy,
                _vehicleState.z,
                _vehicleState.dz,
                _vehicleState.phi,
                _vehicleState.dphi,
                _vehicleState.theta,
                _vehicleState.dtheta,
                _vehicleState.psi,
                _vehicleState.dpsi
            };

            serializer.serializeFloats(MSP_STATE, statevals, 10);

            sendPayload(serializer);
        }

        void sendClosedLoopControlMessage()
        {
            MspSerializer serializer = {};

            _closedLoopControl.serializeMessage(serializer);

            sendPayload(serializer);
        }

        void sendPayload(const MspSerializer & serializer) {
            for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                comms_write_byte(serializer.payload[k]);
            }
        }
 
        void runLed(const status_t status)
        {
            const uint32_t msec_curr = millis();

            if (!_imu.isCalibrated()) {
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
            static bool pulsing;
            static uint32_t pulse_start;

            if (_ledTimer.ready(freq)) {
                led_set(true);
                pulsing = true;
                pulse_start = msec_curr;
            }

            else if (pulsing) {
                if (millis() - pulse_start > LED_PULSE_DURATION_MSEC) {
                    led_set(false);
                    pulsing = false;
                }
            }
        }

        void runCommandParser(command_t & command)
        {
            if (_commandTimer.ready(COMMS_FREQ)) {

                static MspParser _commandParser;

                uint8_t byte = 0;

                while (comms_read_byte(&byte)) {

                    switch (_commandParser.parse(byte)) {

                        case MSP_SET_ARMING:
                            command.armed = !command.armed;
                            break;

                        case MSP_SET_IDLE:
                            command.hovering = false;
                            break;

                        case MSP_SET_SETPOINT:
                            command.hovering = true;
                            command.demands.pitch = _commandParser.getFloat(0);
                            command.demands.roll = _commandParser.getFloat(1);
                            command.demands.yaw = _commandParser.getFloat(2);
                            command.demands.thrust = _commandParser.getFloat(3);
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
