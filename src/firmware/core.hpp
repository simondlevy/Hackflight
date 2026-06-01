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
#include <firmware/flying.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/led.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/message.h>
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

    class Core {

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

        public:

            void beginCore()
            {
                Serial1.begin(115200);

                _imu.begin();
                _led.begin(); 

                _mode = MODE_IDLE;
            }

            void beginHoverDeck()
            {
                _zranger.begin();
                _flowsensor.begin();
            }

            auto updateCore(
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

            auto updateCoreAndHover(
                    const msp_message_t & message,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                step(message.is_armed, message.is_hovering,
                        message.timestamp_msec, motorvals, motorcount);

                updateHoverDeck();

                const auto dt = Timer::getDt();

                _altHoldPid= AltHoldPidController::run(_altHoldPid,
                        dt, _mode, _state, message.setpoint);

                const auto rxaxes = message.setpoint;

                const auto setpoint = Setpoint(
                        rxaxes.thrust < 0 ? 0 : rxaxes.thrust,
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

            void updateHoverDeck()
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

            // Computation
            ImuFilter _imuFilter;
            EKF _ekf;
            FlyingCheck _flyingCheck;

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

            // Hover-deck support
            OpticalFlowFilter _opticalFlowFilter;
            ZRangerFilter _zrangerFilter;

            // PID control for stabilize-only
            StabilizerPidController _stabilizerPid;

            // PID control for altitude-hold
            AltHoldPidController _altHoldPid;

            // PID control for hover
            HoverPidController _hoverPid;

            // Debugging
            Debugger _debugger;

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
                    _ekf = EKF::predict(_ekf, millis(), _flyingCheck.isFlying); 
                }

                // Do EKF fast-update with IMU readings
                _ekf = EKF::update(_ekf, _imuFilter.output, millis());
            
                // Get vehicle state from EKF
                _state = EKF::getVehicleState(_ekf);

                // Periodically run flying check to get status for EKF
                if (_flyingCheckTimer.ready()) {
                    _flyingCheck = FlyingCheck::run(_flyingCheck, millis(),
                            motorvals, motorcount);
                }
            }

            void sendTelemetry(const Setpoint & setpoint)
            {
                if (_telemetryTimer.ready()) {
                    static MspSerializer _serializer;

                    const float data[14] = {
                        setpoint.thrust, setpoint.roll, setpoint.pitch,
                        setpoint.yaw, _state.dx, _state.dy, _state.z, _state.dz,
                        _state.phi, _state.dphi, _state.theta, _state.dtheta,
                        _state.psi, _state.dpsi
                    };

                    _serializer = MspSerializer::serializeFloats(
                            _serializer, MSP_TELEMETRY, data, 14);

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

                    // Panic mode: can't recover
                    mode == MODE_PANIC ? MODE_PANIC :

                    //  Vehicle flipped over: enter panic mode
                    isFlipped(state) ? MODE_PANIC :

                    // Want arm and safe to arm: enter armed mode
                    mode == MODE_IDLE && shouldArm && imufilt.isGyroCalibrated
                    ? MODE_ARMED :

                    // Want disarm: enter idle mode
                    mode == MODE_ARMED && !shouldArm ? MODE_IDLE :

                    // Armed and requested hover; enter hover mode
                    mode == MODE_ARMED && requestedHover ? MODE_HOVERING :

                    // Hovering and requested no-hover; return to armed mode
                    mode == MODE_HOVERING && !requestedHover ? MODE_ARMED :

                    //  Default: stay in current mode
                    mode;
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
    }; // class Core

} // namespace hf
