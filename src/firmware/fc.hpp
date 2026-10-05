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
#include <firmware/debugger.hpp>
#include <firmware/estimator/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>
#include <firmware/msp/serializer.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/receivers/espnow.hpp>
#include <firmware/timer.hpp>
#include <firmware/voltage_divider.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
#include <pidcontrol/hover.hpp>

namespace hf {

    class FlightController {

        // Constants ---------------------------------------------------------

        private:

            // Voltage divider sensing
            static constexpr uint8_t kVoltageInputPin = A9;
            static constexpr float kR1Ohms = 3300;
            static constexpr float kR2Ohms = 1000;

            // LED indicator
            static const uint8_t kLedPin = 9;
            static constexpr float kLedHeartbeat_Rate = 0.75;
            static constexpr float kLedFastBlink_Rate = 3;
            static constexpr uint32_t kLedPulseDurationMsec = 50;

            // Rate constants for timer tasks
            static constexpr float kCoreLoopRate = 1000;
            static constexpr float kEkfPredictionRate = 100;
            static constexpr float kFlyingCheckRate   = 25;
            static constexpr float kVoltageSensingRate = 10;
            static constexpr float kHoverDeckAcquisitionRate = 100;
            static constexpr float kTelemetryRate = 50;

            // Safety constants
            static constexpr float kTiltAngleFlippedMinDeg = 75;
            static constexpr uint32_t kFailsafeMsec = 500;

            // We say we are flying if one or more motors are running
            // over the idle thrust.
            static const uint32_t kFlyingHysteresisThresholdMsec = 2000;
            static constexpr float kMotorIdleMax = 0.1;

            // Public instance methods --------------------------------------------

        public:

            void Begin()
            {
                imu_.Begin();

                pinMode(kLedPin, OUTPUT); 

                zranger_.Begin();
                flow_sensor_.Begin();

                mode_ = kModeIdle;
            }

            auto Update(
                    const EspNowReceiver & rx,
                    HardwareSerial & serial,
                    const float * motor_vals,
                    const uint8_t motor_count) -> Setpoint
            {
                // Run sensor fusion on hover-deck
                RunHoverDeck();

                // Safely update flight mode
                UpdateMode(rx, false); // no hover request

                // Periodically run flying check to get status for EKF
                UpdateFlyingStatus(motor_vals, motor_count);

                // Sense voltage periodically
                UpdateVoltage();

                // Blink LED to indicate status
                BlinkLed(imu_filter_.is_gyro_calibrated && mode_ != kModePanic);

                // Update the IMU filter with raw IMU data
                UpdateImu();

                // Update state estimation
                UpdateState();

                // Convert receiver values into setpoint appropriate for PID
                // controllers
                const auto setpoint = MakeSetpoint(rx);

                // Send setpoint and vehicle state to base-station
                SendTelemetry(serial, setpoint);

                // Run PID controller on sepoint
                return RunPidController(setpoint);
            }

            auto IsSafeToFly() -> bool
            {
                return mode_ != kModePanic;
            }

            auto IsArmed() -> bool
            {
                return mode_ != kModeIdle;
            }

            // Static methods ----------------------------------------------

        private:

            static auto CheckFailsafe(
                    const uint32_t msec_curr,
                    const uint32_t msec_prev,
                    const bool requested_arming) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > kFailsafeMsec;

                return timed_out ? false : requested_arming;
            } 

            static auto IsFlipped(const VehicleState & state) -> bool
            {
                return IsFlippedAngle(state.theta) ||
                    IsFlippedAngle(state.phi); 
            }

            static auto IsFlippedAngle(const float angle) -> bool
            {
                return fabs(angle) > kTiltAngleFlippedMinDeg;
            }

            static auto MakeSetpoint(const EspNowReceiver & rx) -> Setpoint
            {                
                return Setpoint(
                    (EspNowReceiver::GetThrottle(rx)+1)/2, // [-1,+1] => [0,1]
                    PositionController::bypass(EspNowReceiver::GetRoll(rx)),
                    PositionController::bypass(EspNowReceiver::GetPitch(rx)),
                    EspNowReceiver::GetYaw(rx));
            }

            // Instance variables ---------------------------------------------

            // Vehicle state
            VehicleState state_;

            // Idle, armed, etc.
            Mode mode_;

            // Flying status based on motors
            bool is_flying_;
            uint32_t motor_check_msec_;

            // Sensor fusion
            ImuFilter imu_filter_;
            EKF ekf_;
            OpticalFlowFilter optical_flow_filter_;
            ZRangerFilter zranger_filter_;

            // Devices
            IMU imu_;
            ZRanger zranger_;
            OpticalFlowSensor flow_sensor_;

            // Voltage sensing
            float voltage_;

            // Timers
            Timer ekf_prediction_timer_ = Timer(kEkfPredictionRate);
            Timer flying_check_timer_ = Timer(kFlyingCheckRate);
            Timer voltage_sensing_timer_ = Timer(kVoltageSensingRate);
            Timer hover_deck_timer_ = Timer(kHoverDeckAcquisitionRate);
            Timer telemetry_timer_ = Timer(kTelemetryRate);

            // PID control for stabilize-only
            StabilizerPidController stabilizer_pid_;

            // PID control for hover
            HoverPidController hover_pid_;

            // Telemetry serializer
            MspSerializer telemetry_serializer_;

            // Debugging
            Debugger debugger_;

            // Support for GetDt()
            uint32_t usec_prev_;

            // Support for LED blink
            bool is_led_pusing_;
            uint32_t led_pulse_start_;

            // Support for LED blinking
            Timer heartbeat_timer_ = Timer(kLedHeartbeat_Rate);
            Timer fast_blink_timer_ = Timer(kLedFastBlink_Rate);

            // Support for voltage sensing
            VoltageDivider voltage_divider_ = VoltageDivider(
                    kVoltageInputPin, kR1Ohms, kR2Ohms);

            // Instance methods ---------------------------------------------0

            auto AreMotorsAboveIdle(
                    const float * motor_vals,
                    const uint8_t motor_count) -> bool
            {
                auto is_thrust_hover_idle = false;

                for (int i = 0; i < motor_count; ++i) {
                    if (motor_vals[i] > kMotorIdleMax) {
                        is_thrust_hover_idle = true;
                        break;
                    }
                }

                const auto msec_curr = millis();

                motor_check_msec_ = is_thrust_hover_idle ? msec_curr :
                    motor_check_msec_;

                return  motor_check_msec_ > 0 &&
                    (msec_curr - motor_check_msec_) <
                    kFlyingHysteresisThresholdMsec;
            }

            void BlinkLed(const bool isimu__calibrated)
            {
                const auto ready = isimu__calibrated ?
                    heartbeat_timer_.Ready() : fast_blink_timer_.Ready();

                if (ready) {
                    digitalWrite(kLedPin, true);
                    is_led_pusing_ = true;
                    led_pulse_start_ = millis();
                }

                else if (is_led_pusing_) {
                    if (millis() - led_pulse_start_ > kLedPulseDurationMsec) {
                        digitalWrite(kLedPin, false);
                        is_led_pusing_ = false;
                    }
                }
            }

            auto GetDt() -> float
            {
                const auto usec_curr = micros();      
                const float dt = (usec_curr - usec_prev_)/1000000.0;
                usec_prev_ = usec_curr;

                return dt;
            }

            void RunHoverDeck()
            {
                if (hover_deck_timer_.Ready()) {
                    zranger_filter_ = ZRangerFilter::Update(
                            zranger_filter_, zranger_.Read());
                    optical_flow_filter_ = OpticalFlowFilter::Update(
                            optical_flow_filter_,
                            micros(), flow_sensor_.Read());
                    ekf_ = EKF::Update(ekf_, zranger_filter_, optical_flow_filter_);
                }
            }

            auto RunPidController(const Setpoint & setpoint) -> Setpoint
            {
                stabilizer_pid_ = StabilizerPidController::Run(stabilizer_pid_,
                        is_flying_, GetDt(), state_, setpoint);

                return stabilizer_pid_.setpoint;
            }

            void SendTelemetry(
                    HardwareSerial & serial,
                    const Setpoint & setpoint)
            {
                if (telemetry_timer_.Ready()) {

                    float data[256] = {};

                    data[0] = (float)mode_;

                    data[1] = setpoint.thrust;
                    data[2] = setpoint.roll;
                    data[3] = setpoint.pitch;
                    data[4] = setpoint.yaw;

                    data[5] = state_.dx;
                    data[6] = state_.dy;
                    data[7] = state_.z;
                    data[8] = state_.dz;
                    data[9] = state_.phi;
                    data[10] = state_.dphi;
                    data[11] = state_.theta;
                    data[12] = state_.dtheta;
                    data[13] = state_.psi;
                    data[14] = state_.dpsi;

                    telemetry_serializer_ = MspSerializer::SerializeFloats(
                            telemetry_serializer_, kMspTelemetry,
                            data, 15);

                    serial.write(
                            MspSerializer::GetPayloadBytes(telemetry_serializer_),
                            MspSerializer::GetPayloadSize(telemetry_serializer_));
                }
            }

            void UpdateFlyingStatus(
                    const float * motor_vals,
                    const uint8_t motor_count)
            {
                is_flying_ = 

                    mode_ == kModeIdle || mode_ == kModePanic  ? false :

                    flying_check_timer_.Ready() ?
                    AreMotorsAboveIdle(motor_vals, motor_count) :

                    is_flying_;
            }

            void UpdateImu()
            {
                imu_filter_ = ImuFilter::Step(imu_filter_, millis(),
                        imu_.Read(), imu_.GetGyroRangeDps(),
                        imu_.GetAccelRangeGs());
            }

            void UpdateMode(
                    const EspNowReceiver & rx, const bool requested_hover)
            {
                const auto requested_arming = EspNowReceiver::IsArmed(rx);

                const auto is_gyro_calibrated = imu_filter_.is_gyro_calibrated;

                const auto should_arm = 

                    // Disable arming while gyro is calibrating
                    !is_gyro_calibrated ? false :

                    // Check receiver timeout
                    CheckFailsafe(millis(),
                            EspNowReceiver::GetTimestampMsec(rx),
                            requested_arming);

                // Run a little state-transition machine to update flight mode
                mode_ = 

                    //  Vehicle flipped over: enter panic mode
                    IsFlipped(state_) ? kModePanic :

                    // Panic mode: can't recover
                    mode_ == kModePanic ? kModePanic :

                    // Disallow jumping directly from idle to hover
                    mode_ == kModeIdle && requested_hover ? kModeIdle :

                    // Want arm and safe to arm: enter armed mode
                    mode_ == kModeIdle && should_arm && is_gyro_calibrated ?
                    kModeArmed :

                    // Armed and requested disarm: enter idle mode
                    mode_ == kModeArmed && !should_arm ? kModeIdle :

                    // Armed and requested hover; enter hover mode
                    mode_ == kModeArmed && requested_hover ? kModeHovering :

                    // Hovering and requested no-hover; return to armed mode
                    mode_ == kModeHovering && !requested_hover ? kModeArmed :

                    // Hovering and requested disarm; enter idle mode
                    mode_ == kModeHovering && !requested_arming ? kModeIdle :

                    //  Default: stay in current mode
                    mode_;
            }

            void UpdateState()
            {
                // Periodically run the EKF prediction step
                if (ekf_prediction_timer_.Ready()) {
                    ekf_ = EKF::Predict(ekf_, millis(), is_flying_); 
                }

                // Do EKF fast-update with IMU readings
                ekf_ = EKF::Update(ekf_, imu_filter_.output, millis());

                state_ = EKF::getVehicleState(ekf_);
            }

            void UpdateVoltage()
            {
                voltage_ = voltage_sensing_timer_.Ready() ?
                    voltage_divider_.read() : voltage_;
            }

    }; // class FlightController

} // namespace hf
