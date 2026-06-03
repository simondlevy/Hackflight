/*
   Hackflight core flight-control class

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/led.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>
#include <firmware/msp/serializer.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/profiling.hpp>
#include <firmware/receiver.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
#include <firmware/timer.hpp>
#include <pidcontrol/hover.hpp>

namespace hf {

    class FC {

        private:

            // Arbitrary
            static const uint8_t LED_PIN = 9;

            // Rate constants
            static constexpr float EKF_PREDICTION_RATE_HZ = 100;
            static constexpr float FLYING_CHECK_RATE_HZ   = 25;
            static constexpr float HOVER_DECK_ACQUISITION_RATE_HZ = 100;
            static constexpr float TELEMETRY_RATE_HZ = 50;

            // Safety constants
            static constexpr float TILT_ANGLE_FLIPPED_MIN_DEG = 75;
            static constexpr uint32_t FAILSAFE_MSEC = 500;

            // We say we are flying if one or more motors are running over the
            // idle thrust.
            static const uint32_t FLYING_HYSTERESIS_THRESHOLD_MSEC = 2000;
            static constexpr float MOTOR_IDLE_MAX = 0.05;

        public:

            void begin(const bool useHoverdeck=true)
            {
                Serial1.begin(115200);

                _imu.begin();
                _led.begin(); 

                if (useHoverdeck) {
                    _zranger.begin();
                    _flowsensor.begin();
                }

                _mode = MODE_IDLE;
            }

            auto update(
                    const ReceiverData & rxdata,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                step(rxdata.is_armed, false, // false = no hover
                        rxdata.timestamp_msec, motorvals, motorcount);

                const auto rxaxes = rxdata.axes;

                const auto setpoint = Setpoint(
                        (rxaxes.thrust+1)/2,
                        rxaxes.roll * PositionController::MAX_DEMAND_DEG,
                        rxaxes.pitch * PositionController::MAX_DEMAND_DEG, 
                        rxaxes.yaw);

                _stabilizerPid = StabilizerPidController::run(
                        _stabilizerPid,
                        !rxdata.is_throttle_down,
                        Timer::getDt(),
                        _state,
                        setpoint);

                sendTelemetry(_stabilizerPid.setpoint);

                return _stabilizerPid.setpoint;
            } 

            auto update(
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                step(_message_requested_arming, _message_requested_hover,
                        _message_timestamp_msec, motorvals, motorcount);

                acquireHoverData();

                const auto dt = Timer::getDt();

                _altHoldPid= AltHoldPidController::run(_altHoldPid,
                        dt, _mode, _state, _message_setpoint);

                const auto rxaxes = _message_setpoint;

                const auto setpoint = Setpoint(
                        _altHoldPid.thrust,
                        rxaxes.roll * PositionController::MAX_DEMAND_DEG,
                        rxaxes.pitch * PositionController::MAX_DEMAND_DEG, 
                        rxaxes.yaw);

                _stabilizerPid = StabilizerPidController::run(
                        _stabilizerPid,
                        rxaxes.thrust > 0.01,
                        dt,
                        _state,
                        setpoint);

                sendTelemetry(_stabilizerPid.setpoint);

                return _stabilizerPid.setpoint;
            } 

            void acquireHoverData()
            {
                // Slower EKF update with range, optical flow
                if (_hoverDeckTimer.ready()) {
                    _zrangerFilter = ZRangerFilter::update(
                            _zrangerFilter, _zranger.read());
                    _opticalFlowFilter = OpticalFlowFilter::update(
                            _opticalFlowFilter,
                            micros(), _flowsensor.read());
                    _ekf = EKF::update(_ekf, _zrangerFilter, _opticalFlowFilter);
                }
            }

            void handleSerial1Event()
            {
                while (Serial1.available()) {

                    handleIncomingByte(Serial1.read());
                }
            }

            auto isSafeToFly() -> bool
            {
                return _mode != MODE_PANIC;
            }

            auto isArmed() -> bool
            {
                return _mode != MODE_IDLE;
            }

        private:

            // Vehicle state
            VehicleState _state;

            // Idle, armed, etc.
            mode_e _mode;

            // Flying status based on motors
            bool _isFlying;
            uint32_t _flyingCheckMsec;

            // Sensor fusion
            ImuFilter _imuFilter;
            EKF _ekf;
            OpticalFlowFilter _opticalFlowFilter;
            ZRangerFilter _zrangerFilter;

            // Devices
            IMU _imu;
            LED _led = LED(LED_PIN);
            ZRanger _zranger;
            OpticalFlowSensor _flowsensor;

            // Timers
            Timer _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
            Timer _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
            Timer _hoverDeckTimer = Timer(HOVER_DECK_ACQUISITION_RATE_HZ);
            Timer _telemetryTimer = Timer(TELEMETRY_RATE_HZ);

            // PID control for stabilize-only
            StabilizerPidController _stabilizerPid;

            // PID control for altitude-hold
            AltHoldPidController _altHoldPid;

            // PID control for hover
            HoverPidController _hoverPid;

            // Debugging
            Debugger _debugger;

            // Incoming message contents
            uint32_t _message_timestamp_msec;
            bool _message_requested_arming;
            bool _message_requested_hover;
            Setpoint _message_setpoint;

            void step(
                    const bool is_armed,
                    bool is_hovering,
                    const uint32_t timestamp_msec,
                    const float * motorvals,
                    const uint8_t motorcount)
            {
                // Safely update flight mode
                _mode = updateMode(millis(), _state, 
                        _imuFilter.isGyroCalibrated, is_armed, is_hovering,
                        timestamp_msec, _imuFilter, _mode);

                //_debugger.report(_mode);

                // Blink IMU to indicate status
                _led.blink(_imuFilter.isGyroCalibrated);

                // Read the raw IMU data
                const auto imuraw = _imu.read();

                // Filter the raw IMU data
                _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
                        _imu.gyroRangeDps(), _imu.accelRangeGs());

                // Periodically run the EKF prediction step
                if (_ekfPredictionTimer.ready()) {
                    _ekf = EKF::predict(_ekf, millis(), _isFlying); 
                }

                // Do EKF fast-update with IMU readings
                _ekf = EKF::update(_ekf, _imuFilter.output, millis());
            
                // Get vehicle state from EKF
                _state = EKF::getVehicleState(_ekf);

                // Periodically run flying check to get status for EKF
                if (_flyingCheckTimer.ready()) {
                    checkFlyingStatus(motorvals, motorcount);
                }
            }

            void checkFlyingStatus(
                    const float * motorvals, const uint8_t motor_count)
            {
                auto isThrustOverIdle = false;

                for (int i = 0; i < motor_count; ++i) {
                    if (motorvals[i] > MOTOR_IDLE_MAX) {
                        isThrustOverIdle = true;
                        break;
                    }
                }

                const auto msec_curr = millis();

                _flyingCheckMsec = isThrustOverIdle ? msec_curr :
                    _flyingCheckMsec;

                _isFlying = _flyingCheckMsec > 0 &&
                    (msec_curr - _flyingCheckMsec) <
                    FLYING_HYSTERESIS_THRESHOLD_MSEC;
            }

            void sendTelemetry(const Setpoint & setpoint)
            {
                if (_telemetryTimer.ready()) {
                    static MspSerializer _serializer;

                    const float data[15] = {
                        (float)_mode,
                        setpoint.thrust, setpoint.roll, setpoint.pitch,
                        setpoint.yaw, _state.dx, _state.dy, _state.z, _state.dz,
                        _state.phi, _state.dphi, _state.theta, _state.dtheta,
                        _state.psi, _state.dpsi
                    };

                    _serializer = MspSerializer::serializeFloats(
                            _serializer, MSP_TELEMETRY, data, 15);

                    Serial1.write(
                            MspSerializer::payloadBytes(_serializer),
                            MspSerializer::payloadSize(_serializer));
                }
            }

            static auto updateMode(
                    const uint32_t msecCurr,
                    const VehicleState & state,
                    const bool isGyroCalibrated,
                    const bool requestedArming,
                    const bool requestedHover,
                    const uint32_t msecPrev,
                    const ImuFilter & imufilt,
                    const mode_e mode) -> mode_e
            {
                const auto shouldArm = 

                    // Disable arming while gyro is calibrating
                    !isGyroCalibrated ? false :

                    // Check receiver timeout
                    checkFailsafe(msecCurr, msecPrev, requestedArming);

                // Run a little state-transition machine to update flight mode
                return 

                    //  Vehicle flipped over: enter panic mode
                    isFlipped(state) ? MODE_PANIC :

                    // Panic mode: can't recover
                    mode == MODE_PANIC ? MODE_PANIC :

                    // Want arm and safe to arm: enter armed mode
                    mode == MODE_IDLE && shouldArm && imufilt.isGyroCalibrated
                    ? MODE_ARMED :

                    // Armed and requested disarm: enter idle mode
                    mode == MODE_ARMED && !shouldArm ? MODE_IDLE :

                    // Armed and requested hover; enter hover mode
                    mode == MODE_ARMED && requestedHover ? MODE_HOVERING :

                    // Hovering and requested no-hover; return to armed mode
                    mode == MODE_HOVERING && !requestedHover ? MODE_ARMED :

                    // Hovering and requested disarm; enter idle mode
                    mode == MODE_HOVERING && !requestedArming ? MODE_IDLE :

                    //  Default: stay in current mode
                    mode;
            }

            void handleIncomingByte(const uint8_t byte)
            {
                static MspParser _parser;

                _parser = MspParser::parse(_parser, byte);

                switch (MspParser::getid(_parser)) {

                    case MSP_SET_ARMING:
                        _message_requested_arming = !_message_requested_arming;
                        _message_timestamp_msec = millis();
                        break;

                    case MSP_SET_IDLE:
                        _message_requested_hover = false;
                        _message_timestamp_msec = millis();
                        break;

                    case MSP_SET_HOVER:
                        _message_requested_hover = true;
                        _message_setpoint.thrust = MspParser::getFloat(_parser, 0);
                        _message_setpoint.pitch = MspParser::getFloat(_parser, 1); // vx
                        _message_setpoint.roll = MspParser::getFloat(_parser, 2); // vy
                        _message_setpoint.yaw = MspParser::getFloat(_parser, 3);
                        _message_timestamp_msec = millis();
                        break;

                    default:
                        break;
                }
            }

            static auto isFlipped(const VehicleState & state) -> bool
            {
                return isFlippedAngle(state.theta) ||
                    isFlippedAngle(state.phi); 
            }

            static auto isFlippedAngle(const float angle) -> bool
            {
                return fabs(angle) > TILT_ANGLE_FLIPPED_MIN_DEG;
            }

            static auto checkFailsafe(
                    const uint32_t msec_curr,
                    const uint32_t msec_prev,
                    const bool is_armed) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > FAILSAFE_MSEC;

                return timed_out ? false : is_armed;
            } 
    }; // class FC

} // namespace hf
