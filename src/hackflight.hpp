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

#include <VL53L1X.h>
#include <pmw3901.hpp>

#include <__messages__.h>
#include <__pid__.hpp>

#include <debugger.hpp>
#include <ekf.hpp>
#include <imu.hpp>
#include <parser.hpp>
#include <setpoints/manual.hpp>
#include <timer.hpp>
#include <vehicles/crazyflie.hpp>

class Hackflight {

    public:

        void init(
                const uint8_t ledPin,
                const bool isLedInverted,
                HardwareSerial * uart,
                TwoWire * wire,
                SPIClass * spi,
                const uint8_t csPin)
        {
            init(ledPin, isLedInverted, uart);

            zrangerInit(wire);

            if (!_pmw3901.begin(csPin, *spi)) {
                Debugger::error("OpticalFlow");
            }
        }

        void init(
                const uint8_t ledPin,
                const bool isLedInverted,
                HardwareSerial * uart)
        {
            _ekf.init(millis());

            Serial.begin(115200);

            _imu.init();

            motors_init();

            _ledPin = ledPin;
            _isLedInverted = isLedInverted;
            pinMode(_ledPin, OUTPUT);

            _uart = uart;
            _uart->begin(115200);

            _msec_start = millis();
        }

        void task1(const uint8_t motorCount, const mixFun_t mixFun)
        {
            //report();

            static flightMode_t _flightMode;
            static float _motorvals[MAX_MOTOR_COUNT];
            static bool _isFlying;
            static PidControl _pidControl;
            static vehicleState_t _vehicleState;
            static command_t _command;
            static bool _didResetEstimation;
            static Timer _flyingStatusTimer;

            const uint32_t msec = millis() - _msec_start;

            // Sync the core loop to the IMU
            const bool imuIsCalibrated = _imu.step(&_ekf, msec);

            // Set the LED based on current flightMode
            runLed(imuIsCalibrated, _flightMode);

            // Run logging
            runLogger(_vehicleState, _pidControl);

            // Get command
            runCommandParser(_command);

            // Periodically update ekf with flying flightMode
            if (_flyingStatusTimer.ready(FLYING_MODE_FREQ)) {
                _isFlying = isFlyingCheck(msec, motorCount, _motorvals);
            }

            // Run ekf to get vehicle state
            getStateEstimate(_isFlying, _vehicleState, _didResetEstimation);

            // Check for lost contact
            if (_command.timestamp > 0 &&
                    millis() - _command.timestamp > COMMAND_TIMEOUT_MSEC) {
                _flightMode = MODE_PANIC;
            }

            // Check for flipped over
            if (isFlippedAngle(_vehicleState.theta) ||
                    isFlippedAngle(_vehicleState.phi)) {
                _flightMode = MODE_PANIC;
            }

            switch (_flightMode) {

                case MODE_IDLE:
                    if (_command.armed && isSafeTakeoffAngle(_vehicleState.phi) &&
                            isSafeTakeoffAngle(_vehicleState.theta)) {
                        _flightMode = MODE_ARMED;
                    }
                    stopMotors(motorCount, _motorvals);
                    break;

                case MODE_ARMED:
                    if (!_command.armed) {
                        _flightMode = MODE_IDLE;
                    }
                    if (_command.hovering) {
                        _flightMode = MODE_HOVERING;
                    }
                    break;

                case MODE_HOVERING:
                    runPidAndMixer(_command, _vehicleState, motorCount,
                            mixFun, _flightMode, _pidControl, _motorvals);
                    if (!_command.hovering) {
                        _flightMode = MODE_LANDING;
                    }
                    break;

                case MODE_LANDING:
                    runPidAndMixer(_command, _vehicleState, motorCount,
                            mixFun, _flightMode, _pidControl, _motorvals);
                    break;

                case MODE_PANIC:
                    // No way to recover from this
                    stopMotors(motorCount, _motorvals);
                    break;

                default:
                    break;
            }

            for (uint8_t k=0; k<motorCount; ++k) {
                motors_setSpeed(k, _motorvals[k]);
            }
            motors_run();
        }

        void task2()
        {
            zrangerUpdate(_vl53l1x.read());

            int16_t deltaX = 0;
            int16_t deltaY = 0;
            bool gotMotion = false;

            _pmw3901.readMotion(deltaX, deltaY, gotMotion);

            flowMeasurement_t flowData = {};

            if (flowUpdate(deltaX, deltaY, gotMotion, flowData)) {
                _ekf.enqueueFlow(&flowData);
            }
        }

    private:

        // Crucial frequencies
        static constexpr float FLYING_MODE_FREQ = 25;
        static constexpr float COMMS_FREQ = 100;
        static const uint32_t EKF_PREDICTION_FREQ = 100;

        // LED Status indicator
        static constexpr float LED_HEARTBEAT_FREQ = 1;
        static constexpr float LED_IMU_CALIBRATING_FREQ = 3;
        static constexpr uint32_t LED_PULSE_DURATION_MSEC = 50;

        static const uint8_t TASK_PRIORITY = 5;
        static const uint32_t COMMAND_TIMEOUT_MSEC = 100;
        static constexpr float TILT_ANGLE_TAKEOFF_MAX = 30;
        static constexpr float TILT_ANGLE_FLIPPED_MIN = 75;
        static const uint32_t IS_FLYING_HYSTERESIS_THRESHOLD = 2000;
        static const uint8_t MAX_MOTOR_COUNT = 20; // whatevs
        static constexpr float MAX_SAFE_ANGLE = 75;

        static constexpr float MAX_VELOCITY = 10; //meters per second

        static const uint16_t ZRANGER_OUTLIER_LIMIT_MM = 5000;
        static constexpr float ZRANGER_EXP_POINT_A = 2.5;
        static constexpr float ZRANGER_EXP_STD_A = 0.0025; 
        static constexpr float ZRANGER_EXP_POINT_B = 4.0;
        static constexpr float ZRANGER_EXP_STD_B = 0.2;   

        static const int16_t FLOW_OUTLIER_LIMIT = 100;
        static constexpr float FLOW_STD_FIXED = 2.0;


        typedef struct {

            uint32_t timestamp;
            bool armed;
            bool hovering;
            demands_t setpoint;

        } command_t;

        Imu _imu;
        EKF _ekf;
        Debugger _debugger;
        uint32_t _msec_start;

        HardwareSerial * _uart;

        VL53L1X _vl53l1x;
        PMW3901 _pmw3901;

        uint8_t _ledPin;
        bool _isLedInverted;

        void zrangerInit(TwoWire * wire)
        {
            wire->begin();
            wire->setClock(400000);
            delay(100);

            _vl53l1x.setBus(wire);

            if (!_vl53l1x.init()) {
                //Debugger::error("ZRanger");
            }

            _vl53l1x.setDistanceMode(VL53L1X::Medium);
            _vl53l1x.setMeasurementTimingBudget(25000);

            _vl53l1x.startContinuous(50);
        }

        void zrangerUpdate(const float range)
        {
            const float expCoeff =
                logf(ZRANGER_EXP_STD_B / ZRANGER_EXP_STD_A) /
                (ZRANGER_EXP_POINT_B - ZRANGER_EXP_POINT_A);

            // check if range is feasible and push into the ekf the
            // sensor should not be able to measure >5 [m], and outliers
            // typically occur as >8 [m] measurements
            if (range < ZRANGER_OUTLIER_LIMIT_MM) {

                float distance = range / 1000; // Scale from [mm] to [m]

                float stdDev = ZRANGER_EXP_STD_A * (
                        1 + expf(expCoeff * (distance - ZRANGER_EXP_POINT_A)));


                tofMeasurement_t tofData = {};

                tofData.distance = distance;
                tofData.stdDev = stdDev;

                _ekf.enqueueRange(&tofData);
            }
        }

        bool flowUpdate(
                const int16_t deltaX,
                const int16_t deltaY,
                const bool gotMotion,
                flowMeasurement_t & flowData)
        {
            // Flip motion information to comply with sensor mounting
            // (might need to be changed if mounted differently)
            int16_t accpx = -deltaY;
            int16_t accpy = -deltaX;

            // Outlier removal
            if (abs(accpx) < FLOW_OUTLIER_LIMIT && abs(accpy) < FLOW_OUTLIER_LIMIT) {

                static uint32_t _msecPrev;

                // Form flow measurement struct and push into the EKF
                flowData.stdDevX = FLOW_STD_FIXED;
                flowData.stdDevY = FLOW_STD_FIXED;
                flowData.dt = (float)(micros()-_msecPrev)/1000000.0f;

                // We do want to update dt every measurement and not only
                // in the ones with detected motion, as we work with
                // instantaneous gyro and velocity values in the update
                // function (meaning assuming the current measurements over
                // all of dt)
                _msecPrev = micros();

                // Use raw measurements
                flowData.dpixelx = (float)accpx;
                flowData.dpixely = (float)accpy;

                // Push measurements into the ekf if flow is not disabled
                //    and the PMW flow sensor indicates motion detection
                if (gotMotion) {
                    return true;
                }
            }

            return false;
        }

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
            quaternion_t quat = {};

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

        void runPidAndMixer(
                const command_t &command,
                const vehicleState_t & state,
                const uint8_t motorCount,
                const mixFun_t mixFun,
                flightMode_t & flightMode,
                PidControl & control,
                float *motorvals)
        {
            const bool controlled = flightMode == MODE_HOVERING ||
                flightMode == MODE_AUTONOMOUS;

            static Timer _timerSlow;

            static demands_t _demandsSlow;

            if (_timerSlow.ready(PID_SLOW_FREQ)) {

                const float dt = 1.f / PID_SLOW_FREQ;

                const demands_t sp = command.setpoint;

                demands_t setpoint = { sp.thrust, sp.roll, sp.pitch, sp.yaw };

                ManualSetpoint::run(dt, setpoint);

                control.runSlow(dt, controlled, state, setpoint, _demandsSlow);

                if (!command.armed) {
                    flightMode = MODE_IDLE;
                }
            }

            static Timer _timerFast;

            if (_timerFast.ready(PID_FAST_FREQ)) {

                demands_t demandsFast = {};

                control.runFast(1.f / PID_FAST_FREQ, controlled, state,
                        _demandsSlow, demandsFast);

                runMixer(motorCount, mixFun, demandsFast, motorvals);

            }
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

        void runLogger(const vehicleState_t & state, PidControl & control)
        {
            static Timer _timer;

            if (_timer.ready(COMMS_FREQ)) {
                sendVehicleState(state);
                sendPidControlMessage(control);
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

        void sendPidControlMessage(PidControl & control)
        {
            MspSerializer serializer = {};

            control.serializeMessage(serializer);

            sendPayload(serializer);
        }

        void sendPayload(const MspSerializer & serializer) {
            for (uint8_t k=0; k<serializer.payloadSize; ++k) {
                _uart->write(serializer.payload[k]);
            }
        }

        void runLed(const bool imuIsCalibrated, const flightMode_t flightMode)
        {
            const uint32_t msec_curr = millis();

            if (!imuIsCalibrated) {
                blinkLed(msec_curr, LED_IMU_CALIBRATING_FREQ);
            }

            else if (flightMode == MODE_ARMED ||
                    flightMode == MODE_HOVERING || 
                    flightMode == MODE_LANDING) { 
                digitalWrite(_ledPin, !_isLedInverted);
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
                digitalWrite(_ledPin, !_isLedInverted);
                _pulsing = true;
                _pulse_start = msec_curr;
            }

            else if (_pulsing) {
                if (millis() - _pulse_start > LED_PULSE_DURATION_MSEC) {
                    digitalWrite(_ledPin, _isLedInverted);
                    _pulsing = false;
                }
            }
        }

        void runCommandParser(command_t & command)
        {
            static Timer _timer;

            if (_timer.ready(COMMS_FREQ)) {

                static MspParser commandParser;

                while (_uart->available()) {

                    const uint8_t byte = _uart->read();

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
                            command.setpoint.pitch = commandParser.getFloat(0);
                            command.setpoint.roll = commandParser.getFloat(1);
                            command.setpoint.yaw = commandParser.getFloat(2);
                            command.setpoint.thrust = commandParser.getFloat(3);
                            command.timestamp = millis();
                            break;

                        default:
                            break;
                    }
                }
            }
        }

        void stopMotors(const uint8_t motorCount, float * motorvals)
        {
            for (uint8_t k=0; k<motorCount; ++k) {
                motorvals[k] = 0;
            }
        }

        static bool isSafeTakeoffAngle(float angle)
        {
            return fabs(angle) < TILT_ANGLE_TAKEOFF_MAX;
        }

        static bool isFlippedAngle(float angle)
        {
            return fabs(angle) > TILT_ANGLE_FLIPPED_MIN;
        }

        void report()
        {
            static uint32_t _count;
            static Timer _timer;
            if (_timer.ready(1)) {
                if (_count > 0) {
                    Serial.println(_count);
                }
                _count = 0;
            }
            _count++;
        }

        // Device-dependent ---------------------------

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
