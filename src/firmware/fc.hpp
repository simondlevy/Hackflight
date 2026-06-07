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
#include <firmware/datatypes.hpp>
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
#include <firmware/receivers/gamepad.hpp>
#include <firmware/receivers/springy.hpp>
#include <firmware/receivers/traditional.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
#include <pidcontrol/hover.hpp>

namespace hf {

    class FC {

        // Constants ---------------------------------------------------------

        private:

            // Arbitrary
            static const uint8_t LED_PIN = 9;

            // Rate constants
            static constexpr float CORE_LOOP_HZ = 1000;
            static constexpr float EKF_PREDICTION_RATE_HZ = 100;
            static constexpr float FLYING_CHECK_RATE_HZ   = 25;
            static constexpr float HOVER_DECK_ACQUISITION_RATE_HZ = 100;
            static constexpr float TELEMETRY_RATE_HZ = 50;

            // Safety constants
            static constexpr float TILT_ANGLE_FLIPPED_MIN_DEG = 75;
            static constexpr uint32_t FAILSAFE_MSEC = 500;

            // We say we are flying if one or more motors are running
            // over the idle thrust.
            static const uint32_t FLYING_HYSTERESIS_THRESHOLD_MSEC = 2000;
            static constexpr float MOTOR_IDLE_MAX = 0.1;

        // Public instance methods --------------------------------------------

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
                    const TraditionalReceiver & rx,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                const auto rxdata = rx.data;

                step(rxdata.requested_arming, false, // false = no hover
                        rxdata.timestamp_msec, motorvals, motorcount);

                const auto rxsetpoint = rxdata.setpoint;

                const auto setpoint = Setpoint(
                        (rxsetpoint.thrust+1)/2, // [-1,+1] => [0,1]
                        rxsetpoint.roll *
                        PositionController::MAX_DEMAND_DEG,
                        rxsetpoint.pitch *
                        PositionController::MAX_DEMAND_DEG, 
                        rxsetpoint.yaw);

                _stabilizerPid = StabilizerPidController::run( _stabilizerPid,
                        _isFlying, getDt(), _state, setpoint);

                sendTelemetry(_stabilizerPid.setpoint);

                return _stabilizerPid.setpoint;
            } 

            auto update(
                    const SpringyReceiver & rx,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                // Run sensor fusion on hover-deck
                acquireHoverData();

                const auto rxdata = rx.data;

                return update(rxdata.setpoint, rxdata.requested_arming,
                        rxdata.requested_hover, rxdata.timestamp_msec,
                        motorvals, motorcount);
            } 

            auto update(
                    GamepadReceiver & gamepad,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                // Run sensor fusion on hover-deck
                acquireHoverData();

                const auto gpdata = gamepad.data;

                return update(gpdata.setpoint, gpdata.requested_arming,
                        gpdata.requested_hover, gpdata.timestamp_msec,
                        motorvals, motorcount);
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

            auto isSafeToFly() -> bool
            {
                return _mode != MODE_PANIC;
            }

            auto isArmed() -> bool
            {
                return _mode != MODE_IDLE;
            }

        private:

            // Static methods ------------------------------------------------

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

                    // Disallow jumping directly from idle to hover
                    mode == MODE_IDLE && requestedHover ? MODE_IDLE :

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
                    const bool requested_arming) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > FAILSAFE_MSEC;

                return timed_out ? false : requested_arming;
            } 

            static void runDelayLoop(const uint32_t usec_curr, 
                    const uint32_t loop_freq_hz)
            {
                float invFreq = 1.0 / loop_freq_hz * 1000000.0;
                uint32_t checker = micros();

                while (invFreq > (checker - usec_curr)) {
                    checker = micros();
                }
            }

            // Instance variables ---------------------------------------------

            // Vehicle state
            VehicleState _state;

            // Idle, armed, etc.
            mode_e _mode;

            // Flying status based on motors
            bool _isFlying;
            uint32_t _motorCheckMsec;

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

            // PID control for hover
            HoverPidController _hoverPid;

            // Telemetry serializer
            MspSerializer _telemetrySerializer;

            // Debugging
            Debugger _debugger;

            // Support for getDt()
            uint32_t _usec_prev;

            // Instance methods ---------------------------------------------0

            auto update(
                    const Setpoint & setpoint_in,
                    const bool requested_arming,
                    const bool requested_hover,
                    const uint32_t timestamp_msec,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                runDelayLoop(micros(), CORE_LOOP_HZ);

                step(requested_arming, requested_hover,
                        timestamp_msec, motorvals, motorcount);

                acquireHoverData();

                _hoverPid= HoverPidController::run(_hoverPid,
                        getDt(), _mode, _state, setpoint_in);

                const auto setpoint_out = _hoverPid.setpoint;

                sendTelemetry(setpoint_out);

                return setpoint_out;
             } 

            void step(
                    const bool requested_arming,
                    bool requested_hover,
                    const uint32_t timestamp_msec,
                    const float * motorvals,
                    const uint8_t motorcount)
            {
                // Safely update flight mode
                _mode = updateMode(millis(), _state,
                        _imuFilter.isGyroCalibrated, requested_arming,
                        requested_hover, timestamp_msec, _imuFilter, _mode);

                // Periodically run flying check to get status for EKF
                _isFlying = 

                    _mode == MODE_IDLE || _mode == MODE_PANIC  ? false :

                    _flyingCheckTimer.ready() ?
                    areMotorsAboveIdle(motorvals, motorcount) :

                    _isFlying;

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
            }

            auto areMotorsAboveIdle(
                    const float * motorvals,
                    const uint8_t motorcount) -> bool
            {
                auto isThrustOverIdle = false;

                for (int i = 0; i < motorcount; ++i) {
                    if (motorvals[i] > MOTOR_IDLE_MAX) {
                        isThrustOverIdle = true;
                        break;
                    }
                }

                const auto msec_curr = millis();

                _motorCheckMsec = isThrustOverIdle ? msec_curr :
                    _motorCheckMsec;

                return  _motorCheckMsec > 0 &&
                    (msec_curr - _motorCheckMsec) <
                    FLYING_HYSTERESIS_THRESHOLD_MSEC;
            }

            void sendTelemetry(const Setpoint & setpoint)
            {
                if (_telemetryTimer.ready()) {

                    const float data[15] = {
                        (float)_mode,
                        setpoint.thrust, setpoint.roll, setpoint.pitch,
                        setpoint.yaw, _state.dx, _state.dy, _state.z, _state.dz,
                        _state.phi, _state.dphi, _state.theta, _state.dtheta,
                        _state.psi, _state.dpsi
                    };

                    _telemetrySerializer = MspSerializer::serializeFloats(
                            _telemetrySerializer, MSP_TELEMETRY, data, 15);

                    Serial1.write(
                            MspSerializer::payloadBytes(_telemetrySerializer),
                            MspSerializer::payloadSize(_telemetrySerializer));
                }
            }

            auto getDt() -> float
            {
                const auto usec_curr = micros();      
                const float dt = (usec_curr - _usec_prev)/1000000.0;
                _usec_prev = usec_curr;

                return dt;
            }

    }; // class FC

} // namespace hf
