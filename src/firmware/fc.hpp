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
#include <firmware/receiver.hpp>
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
            static constexpr float kLedHeartbeatRate = 0.75;
            static constexpr float kLedFastBlinkRate = 3;
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

            typedef enum {

                kLedUnchanged, 
                kLedOff, 
                kLedOn, 

            } LedStatus;

            void Begin()
            {
                imu_.Begin();

                zranger_.Begin();
                flow_sensor_.Begin();

                mode_ = kModeIdle;
            }

            auto Update(
                    const uint32_t usec,
                    const uint16_t rawvolts,
                    const Receiver & rx,
                    const float * motor_vals,
                    const uint8_t motor_count) -> Setpoint
            {

                // Most routines use milliseconds 
                const auto msec = usec / 1000;

                // Run sensor fusion on hover-deck
                RunHoverDeck(usec);

                // Safely update flight mode
                UpdateMode(msec, rx, false); // no hover request

                // Periodically run flying check to get status for EKF
                UpdateFlyingStatus(msec, motor_vals, motor_count);

                // Sense voltage periodically
                UpdateVoltage(msec, rawvolts);

                // Blink LED to indicate status
                UpdateLedStatus(msec, imu_filter_.is_gyro_calibrated && mode_ !=
                        kModePanic);

                // Update the IMU filter with raw IMU data
                UpdateImu(msec);

                // Update state estimation
                UpdateState(msec);

                // Run PID controller on sepoint made from receiver values
                return RunPidController(usec, MakeSetpoint(rx));
            }

            auto IsSafeToFly() -> bool
            {
                return mode_ != kModePanic;
            }

            auto IsArmed() -> bool
            {
                return mode_ != kModeIdle;
            }

            auto IsTelemetryReady(const uint32_t msec) -> bool
            {
                telemetry_timer_ = Timer::Update(telemetry_timer_, 
                        kTelemetryRate, msec);

                return Timer::IsReady(telemetry_timer_);
            }

            auto GetTelemetryBytes(const Receiver & rx) -> TelemetryBytes
            {
                const auto setpoint = MakeSetpoint(rx);

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

                return TelemetryBytes(
                        MspSerializer::GetPayloadBytes(telemetry_serializer_),
                        MspSerializer::GetPayloadSize(telemetry_serializer_));
            }

            auto GetLedStatus() -> LedStatus
            {
                return led_status_;
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

            static auto MakeSetpoint(const Receiver & rx) -> Setpoint
            {                
                return Setpoint(
                        (Receiver::GetThrottle(rx)+1)/2, // [-1,+1] => [0,1]
                        PositionController::bypass(Receiver::GetRoll(rx)),
                        PositionController::bypass(Receiver::GetPitch(rx)),
                        Receiver::GetYaw(rx));
            }

            // Instance variables ---------------------------------------------

            // Vehicle state
            VehicleState state_;

            // Idle, armed, etc.
            Mode mode_;

            // LED status
            LedStatus led_status_;

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
            Timer ekf_prediction_timer_;
            Timer flying_check_timer_;
            Timer voltage_sensing_timer_;
            Timer hover_deck_timer_;
            Timer telemetry_timer_; 

            // PID control for stabilize-only
            StabilizerPidController stabilizer_pid_;

            // PID control for hover
            HoverPidController hover_pid_;

            // Telemetry serializer
            MspSerializer telemetry_serializer_;

            // Debugging
            Debugger debugger_;

            // Support for microsecond PID control timing
            uint32_t usec_prev_;

            // Support for LED blink
            bool is_led_pusing_;
            uint32_t led_pulse_start_;

            // Support for LED blinking
            Timer heartbeat_timer_; 
            Timer fast_blink_timer_;

            // Support for voltage sensing
            VoltageDivider voltage_divider_ =
                VoltageDivider( kR1Ohms, kR2Ohms);

            // Instance methods ----------------------------------------------

            auto AreMotorsAboveIdle(
                    const uint32_t msec,
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

                motor_check_msec_ = is_thrust_hover_idle ? msec :
                    motor_check_msec_;

                return  motor_check_msec_ > 0 &&
                    (msec - motor_check_msec_) <
                    kFlyingHysteresisThresholdMsec;
            }

            void UpdateLedStatus(
                    const uint32_t msec,
                    const bool is_imu__calibrated)
            {
                heartbeat_timer_ = Timer::Update(heartbeat_timer_, 
                        kLedHeartbeatRate, msec);

                fast_blink_timer_ = Timer::Update(fast_blink_timer_, 
                        kLedFastBlinkRate, msec);

                const auto ready =
                    is_imu__calibrated ?
                    Timer::IsReady(heartbeat_timer_) :
                    Timer::IsReady(fast_blink_timer_);

                if (ready) {
                    led_status_ = kLedOn;
                    is_led_pusing_ = true;
                    led_pulse_start_ = msec;
                }

                else if (is_led_pusing_) {
                    if (msec - led_pulse_start_ > kLedPulseDurationMsec) {
                        led_status_ = kLedOff;
                        is_led_pusing_ = false;
                    }
                }

                else {
                    led_status_ = kLedUnchanged;
                }
            }

            void RunHoverDeck(const uint32_t usec)
            {
                const auto msec = usec / 1000;

                hover_deck_timer_ = Timer::Update(hover_deck_timer_,
                        kHoverDeckAcquisitionRate, msec);

                if (Timer::IsReady(hover_deck_timer_)) {
                    zranger_filter_ = ZRangerFilter::Update(
                            zranger_filter_, zranger_.Read());
                    optical_flow_filter_ = OpticalFlowFilter::Update(
                            optical_flow_filter_,
                            usec, flow_sensor_.Read());
                    ekf_ = EKF::Update(ekf_, zranger_filter_, optical_flow_filter_);
                }
            }

            auto RunPidController(const uint32_t usec,
                    const Setpoint & setpoint) -> Setpoint
            {
                const float dt = (usec - usec_prev_)/1000000.0;
                usec_prev_ = usec;

                stabilizer_pid_ = StabilizerPidController::Run(stabilizer_pid_,
                        is_flying_, dt, state_, setpoint);

                return stabilizer_pid_.setpoint;
            }

            void UpdateFlyingStatus(
                    const uint32_t msec,
                    const float * motor_vals,
                    const uint8_t motor_count)
            {
                flying_check_timer_ = Timer::Update(flying_check_timer_,
                        kFlyingCheckRate, msec);

                is_flying_ = 

                    mode_ == kModeIdle || mode_ == kModePanic  ? false :

                    Timer::IsReady(flying_check_timer_) ?
                    AreMotorsAboveIdle(msec, motor_vals, motor_count) :

                    is_flying_;
            }

            void UpdateImu(const uint32_t msec)
            {
                imu_filter_ = ImuFilter::Step(imu_filter_, msec,
                        imu_.Read(), imu_.GetGyroRangeDps(),
                        imu_.GetAccelRangeGs());
            }

            void UpdateMode(const uint32_t msec,
                    const Receiver & rx, const bool requested_hover)
            {
                const auto requested_arming = Receiver::IsArmed(rx);

                const auto is_gyro_calibrated = imu_filter_.is_gyro_calibrated;

                const auto should_arm = 

                    // Disable arming while gyro is calibrating
                    !is_gyro_calibrated ? false :

                    // Check receiver timeout
                    CheckFailsafe(msec,
                            Receiver::GetTimestampMsec(rx),
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

            void UpdateState(const uint32_t msec)
            {
                ekf_prediction_timer_ = Timer::Update(ekf_prediction_timer_,
                        kEkfPredictionRate, msec);

                // Periodically run the EKF prediction step
                if (Timer::IsReady(ekf_prediction_timer_)) {
                    ekf_ = EKF::Predict(ekf_, msec, is_flying_); 
                }

                // Do EKF fast-update with IMU readings
                ekf_ = EKF::Update(ekf_, imu_filter_.output, msec);

                state_ = EKF::getVehicleState(ekf_);
            }

            void UpdateVoltage(const uint32_t msec, const uint16_t rawval)
            {
                voltage_sensing_timer_ = Timer::Update(voltage_sensing_timer_,
                        kVoltageSensingRate, msec);

                voltage_ = Timer::IsReady(voltage_sensing_timer_) ?
                    voltage_divider_.Convert(rawval) : voltage_;
            }

    }; // class FlightController

} // namespace hf
