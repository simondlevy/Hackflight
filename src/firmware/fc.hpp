/*
   Hackflight core flight-control algorithm

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
#include <firmware/safety.hpp>
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
                const auto rxaxes = rxdata.axes;

                step(rxdata.is_armed, rxdata.timestamp_msec, rxaxes,
                        motorvals, motorcount);

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

                return _stabilizerPid.setpoint;
            } 

            auto updateCore(
                    const msp_message_t & message,
                    const float * motorvals,
                    const uint8_t motorcount) -> Setpoint
            {
                step(message.is_armed, message.timestamp_msec,
                        message.setpoint, motorvals, motorcount);

                return Setpoint(0, 0, 0, 0); // XXX
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

            // PID control forhaltitude-hold and stabilize
            AltHoldPidController _altHoldPid;

            // Debugging
            Debugger _debugger;

            void step(
                    const bool is_armed,
                    const uint32_t timestamp_msec,
                    const Setpoint & setpoint,
                    const float * motorvals,
                    const uint8_t motorcount)
            {
                // Run safety checks
                _mode = Safety::updateMode(millis(), _state, 
                        _imuFilter.isGyroCalibrated, is_armed,
                        timestamp_msec, _imuFilter, _mode);

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

                // Periodically send telemetry to client
                if (_telemetryTimer.ready()) {
                    static MspSerializer _serializer;
                    _serializer = MspSerializer::serializeFloats(
                            _serializer, MSP_TELEMETRY, (float *)&_state, 10);
                    Serial1.write(
                            MspSerializer::payloadBytes(_serializer),
                            MspSerializer::payloadSize(_serializer));
                }
            }

    }; // class FC

} // namespace hf
