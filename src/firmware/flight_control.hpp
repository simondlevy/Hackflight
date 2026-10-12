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

#include <vector>

#include <hackflight.h>
#include <firmware/estimator/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/rawdata.hpp>
#include <firmware/led.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>
#include <firmware/msp/serializer.hpp>
#include <firmware/optical_flow.hpp>
#include <firmware/timer.hpp>
#include <firmware/voltage_divider.hpp>
#include <firmware/zranger.hpp>
#include <pidcontrol.hpp>

namespace hf {

    class FlightController {

        private:

            // Voltage divider sensing
            static constexpr float kR1Ohms = 3300;
            static constexpr float kR2Ohms = 1000;

            // LED indicator
            static constexpr float kLedHeartbeatRate = 0.75;
            static constexpr float kLedFastBlinkRate = 3;
            static constexpr uint32_t kLedPulseDurationMsec = 50;

            // Rate constants for timer tasks
            static constexpr float kEkfPredictionRate = 100;
            static constexpr float kFlyingCheckRate   = 25;
            static constexpr float kVoltageSensingRate = 10;
            static constexpr float kTelemetryRate = 50;
            static constexpr float kHoverDeckRate = 100;

            // Safety constants
            static constexpr float kThrottleDownMax = -0.90;
            static constexpr float kTiltAngleFlippedMinDeg = 75;
            static constexpr uint32_t kFailsafeMsec = 500;
            
            // Stick deadband
            static constexpr float kAxisDeadband = 0.02;

            // Flying status based on motors spins
            static constexpr float kMotorIdleMax = 0.1;
            static const uint32_t kFlyingHysteresisThresholdMsec = 2000;

            class FlyingStatus {

                public:

                    FlyingStatus(
                            const bool is_flying,
                            const bool last_flying_check_msec)
                        : is_flying_(is_flying),
                        last_flying_check_msec_(last_flying_check_msec) {}

                    FlyingStatus() = default;

                    FlyingStatus(const FlyingStatus &other) = default;

                    static auto Update(
                            const FlyingStatus & fs,
                            const uint32_t msec,
                            const std::vector<float> motorvals) -> FlyingStatus
                    {
                        auto is_thrust_hover_idle = false;

                        for (auto motorval : motorvals) {
                            if (motorval > kMotorIdleMax) {
                                is_thrust_hover_idle = true;
                                break;
                            }
                        }

                        const auto last_flying_check_msec =
                            is_thrust_hover_idle ? msec :
                            fs.last_flying_check_msec_;

                        const auto is_flying = last_flying_check_msec > 0 &&
                            (msec - last_flying_check_msec) <
                            kFlyingHysteresisThresholdMsec;

                        return FlyingStatus(last_flying_check_msec, is_flying);
                    }

                    bool is_flying_;
                    uint32_t last_flying_check_msec_;
            };

        public:

            FlightController() = default;

            FlightController(const FlightController & other) = default;

            FlightController(const FlightController & fc,
                    const EKF & ekf,
                    const OpticalFlowFilter & optical_flow_filter,
                    const ZRangerFilter & zranger_filter)
                :
                    ekf_(ekf),
                    optical_flow_filter_(optical_flow_filter),
                    zranger_filter_(zranger_filter),
                    state_(fc.state_),
                    mode_(fc.mode_),
                    flying_status_(fc.flying_status_),
                    voltage_(fc.voltage_),
                    ekf_prediction_timer_(fc.ekf_prediction_timer_),
                    flying_check_timer_(fc.flying_check_timer_),
                    voltage_sensing_timer_(fc.voltage_sensing_timer_),
                    telemetry_timer_(fc.telemetry_timer_), 
                    hover_timer_(fc.hover_timer_),
                    heartbeat_timer_(fc.heartbeat_timer_), 
                    fast_blink_timer_(fc.fast_blink_timer_),
                    pid_controller_(fc.pid_controller_),
                    pid_update_usec_prev_(fc.pid_update_usec_prev_),
                    led_(fc.led_),
                    was_arming_button_up_(fc.was_arming_button_up_),
                    receiver_parser_(fc.receiver_parser_),
                    receiver_message_msec_(fc.receiver_message_msec_) {}

            FlightController(
                    const EKF & ekf,
                    const OpticalFlowFilter & optical_flow_filter,
                    const ZRangerFilter & zranger_filter,
                    const ImuFilter & imu_filter,
                    const VehicleState & state,
                    const Mode & mode,
                    const FlyingStatus & flying_status,
                    const float voltage,
                    const Timer & ekf_prediction_timer,
                    const Timer & flying_check_timer,
                    const Timer & voltage_sensing_timer,
                    const Timer & telemetry_timer, 
                    const Timer & hover_timer,
                    const Timer & heartbeat_timer, 
                    const Timer & fast_blink_timer,
                    const PidController & pid_controller,
                    const uint32_t pid_update_usec_prev,
                    const Led & led,
                    const bool was_arming_button_up,
                    const MspParser & receiver_parser,
                    const uint32_t receiver_message_msec)
                        :
                            ekf_(ekf),
                            optical_flow_filter_(optical_flow_filter),
                            zranger_filter_(zranger_filter),
                            imu_filter_(imu_filter),
                            state_(state),
                            mode_(mode),
                            flying_status_(flying_status),
                            voltage_(voltage),
                            ekf_prediction_timer_(ekf_prediction_timer),
                            flying_check_timer_(flying_check_timer),
                            voltage_sensing_timer_(voltage_sensing_timer),
                    telemetry_timer_(telemetry_timer), 
                    hover_timer_(hover_timer),
                    heartbeat_timer_(heartbeat_timer), 
                    fast_blink_timer_(fast_blink_timer),
                    pid_controller_(pid_controller),
                    pid_update_usec_prev_(pid_update_usec_prev),
                    led_(led),
                    was_arming_button_up_(was_arming_button_up),
                    receiver_parser_(receiver_parser),
                    receiver_message_msec_(receiver_message_msec) {}

            static auto ParseReceiverByte(
                    const FlightController & fc,
                    const uint8_t byte,
                    const uint32_t msec)
            {
                auto parser = MspParser::Parse(fc.receiver_parser_, byte);

                const auto got_new_message =
                    MspParser::GetId(parser) == kMspSetChannels;

                const auto timestamp_msec =
                    got_new_message ? msec : fc.receiver_message_msec_;

                return FlightController(
                        fc.ekf_,
                        fc.optical_flow_filter_,
                        fc.zranger_filter_,
                        fc.imu_filter_,
                        fc.state_,
                        fc.mode_,
                        fc.flying_status_,
                        fc.voltage_,
                        fc.ekf_prediction_timer_,
                        fc.flying_check_timer_,
                        fc.voltage_sensing_timer_,
                        fc.telemetry_timer_, 
                        fc.hover_timer_,
                        fc.heartbeat_timer_, 
                        fc.fast_blink_timer_,
                        fc.pid_controller_,
                        fc.pid_update_usec_prev_,
                        fc.led_,
                        fc.was_arming_button_up_,
                        parser,
                        timestamp_msec);
            }

            static auto Update(
                    const FlightController & fc,
                    const uint32_t usec,
                    const IMU::RawData imu_data,
                    const uint16_t rawvolts,
                    const std::vector<float> motorvals, 
                    const PidController::ControlLevel
                    control_level) -> FlightController
            {
                // Most updates run on milliseconds
                const auto msec = usec / 1000;

                return FlightController(

                        UpdateStateEstimator(fc, msec),

                        fc.optical_flow_filter_,

                        fc.zranger_filter_,

                        UpdateImuFilter(fc, msec, imu_data),

                        EKF::getVehicleState(fc.ekf_),

                        UpdateMode(fc, msec, control_level),

                        UpdateFlyingStatus(fc, msec, motorvals),

                        UpdateVoltage(fc, msec, rawvolts),

                        Timer::Update(fc.ekf_prediction_timer_,
                            kEkfPredictionRate, msec),

                        Timer::Update(fc.flying_check_timer_,
                                kFlyingCheckRate, msec),

                        Timer::Update(fc.voltage_sensing_timer_,
                                kVoltageSensingRate, msec),

                        Timer::Update(fc.telemetry_timer_, 
                                kTelemetryRate, msec),

                        Timer::Update(fc.hover_timer_,
                                kHoverDeckRate, msec),

                        Timer::Update(fc.heartbeat_timer_, 
                                kLedHeartbeatRate, msec),

                        Timer::Update(fc.fast_blink_timer_, 
                                kLedFastBlinkRate, msec),

                        UpdatePidController(fc, usec, control_level),

                        usec,

                        UpdateLed(fc, msec,fc.imu_filter_.is_gyro_calibrated
                                && fc.mode_ != kModePanic),

                        !IsArmingButtonDown(fc),

                        fc.receiver_parser_,

                        fc.receiver_message_msec_);
            }

            static auto UpdateHover(
                    const FlightController & fc,
                    const uint32_t usec,
                    const float zdistance,
                    const OpticalFlowData flow) -> FlightController
            {
                const auto zranger_filter = ZRangerFilter::Update(
                        fc.zranger_filter_, zdistance);

                const auto optical_flow_filter = OpticalFlowFilter::Update(
                        fc.optical_flow_filter_, usec, flow);

                const auto ekf = EKF::Update(
                        fc.ekf_, zranger_filter, optical_flow_filter);

                return FlightController(fc, ekf, optical_flow_filter,
                        zranger_filter);
            }

            static auto GetTelemetryBytes(
                    const FlightController &fc) -> std::vector<uint8_t>
            {
                const auto setpoint = MakeSetpoint(fc);

                float data[16] = {

                    (float)fc.mode_,

                    fc.voltage_,

                    setpoint.thrust,
                    setpoint.roll,
                    setpoint.pitch,
                    setpoint.yaw,

                    fc.state_.dx,
                    fc.state_.dy,
                    fc.state_.z,
                    fc.state_.dz,
                    fc.state_.phi,
                    fc.state_.dphi,
                    fc.state_.theta,
                    fc.state_.dtheta,
                    fc.state_.psi,
                    fc.state_.dpsi
                };

                MspSerializer telemetry_serializer;

                telemetry_serializer = MspSerializer::SerializeFloats(
                        telemetry_serializer, kMspTelemetry,
                        data, 16);

                return MspSerializer::GetPayload(telemetry_serializer);
            }

            static auto GetSetpoint(const FlightController & fc) -> Setpoint
            {
                return PidController::GetSetpoint(fc.pid_controller_);
            }

            static auto GetLedStatus(
                    const FlightController & fc) -> Led::Status
            {
                return Led::GetStatus(fc.led_);
            }

            static auto IsSafeToFly(const FlightController & fc) -> bool
            {
                return fc.mode_ != kModePanic;
            }

            static auto IsArmed(const FlightController & fc) -> bool
            {
                return fc.mode_ != kModeIdle;
            }

            static auto ShouldSendTelemetry(
                    const FlightController & fc) -> bool
            {
                return Timer::IsReady(fc.telemetry_timer_);
            }

            static auto ShouldUpdateHover(
                    const FlightController & fc) -> bool
            {
                return Timer::IsReady(fc.hover_timer_);
            }

        private:

            static auto CheckFailsafe(
                    const uint32_t msec_curr,
                    const uint32_t msec_prev,
                    const bool is_arming_button_down) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > kFailsafeMsec;

                return timed_out ? false : is_arming_button_down;
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

            static auto MakeSetpoint(const FlightController & fc) -> Setpoint
            {                
                return Setpoint(
                        GetThrottle(fc),
                        GetRoll(fc),
                        GetPitch(fc),
                        GetYaw(fc));
            }

            static auto UpdateFlyingStatus(
                    const FlightController & fc,
                    const uint32_t msec,
                    const std::vector<float> motorvals) -> FlyingStatus
            {
                return 

                    fc.mode_ == kModeIdle || fc.mode_ == kModePanic ? 
                    FlyingStatus(false, 0) :

                    Timer::IsReady(fc.flying_check_timer_) ?
                    FlyingStatus::Update(fc.flying_status_, msec, motorvals) :

                    fc.flying_status_;
            }

            static auto UpdateLed(
                    const FlightController & fc,
                    const uint32_t msec,
                    const bool is_imu__calibrated) -> Led
            {
                return Led::Update(fc.led_, msec, 
                        is_imu__calibrated ?
                        Timer::IsReady(fc.heartbeat_timer_) :
                        Timer::IsReady(fc.fast_blink_timer_));
            }

            static auto UpdateImuFilter(
                    const FlightController & fc,
                    const uint32_t msec, 
                    const IMU::RawData data)-> ImuFilter
            {
                return ImuFilter::Step(fc.imu_filter_, msec, data);
            }

            static auto UpdateStateEstimator(const FlightController & fc,
                    const uint32_t msec) -> EKF
            {
                auto ekf = fc.ekf_;

                // Periodically run the EKF prediction step
                if (Timer::IsReady(fc.ekf_prediction_timer_)) {
                    ekf = EKF::Predict(ekf, msec, IsFlying(fc));
                }

                // Do EKF fast-update with IMU readings
                return EKF::Update(ekf, fc.imu_filter_.output, msec);
            }

            static auto UpdatePidController(
                    const FlightController & fc,
                    const uint32_t usec,
                    const PidController::ControlLevel
                    control_level) -> PidController
            {
                const float dt = (usec - fc.pid_update_usec_prev_)/1000000.0;

                return PidController::Run(
                        fc.pid_controller_,
                        IsFlying(fc),
                        dt,
                        fc.mode_,
                        fc.state_,
                        MakeSetpoint(fc),
                        control_level);
            }

            static auto UpdateVoltage(
                    const FlightController &fc,
                    const uint32_t msec, const uint16_t rawval) -> float
            {
                return Timer::IsReady(fc.voltage_sensing_timer_) ?
                    VoltageDivider::Convert(kR1Ohms, kR2Ohms, rawval) :
                    fc.voltage_;
            }

            static auto UpdateMode(
                    const FlightController & fc,
                    const uint32_t msec,
                    const PidController::ControlLevel control_level) -> Mode
            {
                const auto is_arming_button_down = IsArmingButtonDown(fc);

                const auto is_gyro_calibrated = fc.imu_filter_.is_gyro_calibrated;

                const auto is_safe_to_arm = 

                    // Disable arming while gyro is calibrating
                    !is_gyro_calibrated ? false :

                    // Check receiver timeout
                    !CheckFailsafe(msec,
                            fc.receiver_message_msec_,
                            is_arming_button_down) ? false :

                    // Throttle must be down for arming in stabilization-only
                    control_level == PidController::kControlStabilize ? 
                    GetThrottle(fc) < kThrottleDownMax :

                    true;

                // Run a little state-transition machine to update flight mode
                return

                    //  Vehicle flipped over: enter panic mode
                    IsFlipped(fc.state_) ? kModePanic :

                    // Panic mode: can't recover
                    fc.mode_ == kModePanic ? kModePanic :

                    // Disallow jumping directly from idle to hover
                    //fc.mode_ == kModeIdle && requested_hover ? kModeIdle :

                    // Want arm and safe to arm: enter armed mode
                    fc.mode_ == kModeIdle &&
                    is_arming_button_down &&
                    fc.was_arming_button_up_ &&
                    is_safe_to_arm ?
                    kModeArmed :

                    // Armed and requested disarm: enter idle mode
                    fc.mode_ == kModeArmed && !is_arming_button_down ? kModeIdle :

                    // Armed and requested hover; enter hover mode
                    //fc.mode_ == kModeArmed && requested_hover ? kModeHovering :

                    // Hovering and requested no-hover; return to armed mode
                    //fc.mode_ == kModeHovering && !requested_hover ? kModeArmed :

                    // Hovering and requested disarm; enter idle mode
                    fc.mode_ == kModeHovering && !is_arming_button_down ? kModeIdle :

                    //  Default: stay in current mode
                    fc.mode_;
            }

            static auto IsFlying(const FlightController & fc) -> bool
            {
                return fc.flying_status_.is_flying_;
            }

            // Special treatment for throttle for now
            static auto GetThrottle(const FlightController & fc) -> float
            {
                const auto val = MspParser::GetShort(fc.receiver_parser_, 0);
                return 2 * (val / 4095.f ) - 1;
            }

            static auto GetRoll(const FlightController & fc) -> float
            {
                return GetAxisValue(fc.receiver_parser_, 1);
            }

            static auto GetPitch(const FlightController & fc) -> float
            {
                return GetAxisValue(fc.receiver_parser_, 2);
            }

            static auto GetYaw(const FlightController & fc) -> float
            {
                return GetAxisValue(fc.receiver_parser_, 3);
            }

            static auto GetReceiverMessageTimestampMsec(
                    const FlightController & fc) -> uint32_t
            {
                return fc.receiver_message_msec_;
            }

            static auto IsArmingButtonDown(const FlightController & fc) -> bool
            {
                return GetSwitchStatus(fc.receiver_parser_, 4);
            }

            static auto IsHoverButtonDown(const FlightController & fc) -> bool
            {
                return GetSwitchStatus(fc.receiver_parser_, 5);
            }

            static auto IsAutopilotButtonDown(const FlightController & fc) -> bool
            {
                return GetSwitchStatus(fc.receiver_parser_, 6);
            }

            static auto GetAxisValue(
                    const MspParser & parser, const uint8_t index) -> float
            {
                const float val = MspParser::GetShort(parser, index) / 2048.f;

                return fabs(val) < kAxisDeadband ? 0 : val;
            }

            static auto GetSwitchStatus(
                    const MspParser & parser, const uint8_t index) -> float
            {
                return MspParser::GetShort(parser, index) > 0;
            }

            // ---------------------------------------------------------------

            // Sensor fusion
            EKF ekf_;
            OpticalFlowFilter optical_flow_filter_;
            ZRangerFilter zranger_filter_;
            ImuFilter imu_filter_;

            // Vehicle state
            VehicleState state_;

            // Idle, armed, etc.
            Mode mode_;

            // Flying status based on motors
            FlyingStatus flying_status_;

            // Voltage sensing
            float voltage_;

            // Timers
            Timer ekf_prediction_timer_;
            Timer flying_check_timer_;
            Timer voltage_sensing_timer_;
            Timer telemetry_timer_; 
            Timer hover_timer_;
            Timer heartbeat_timer_; 
            Timer fast_blink_timer_;

            // PID control for stabilize-only
            PidController pid_controller_;

            // Support for microsecond PID control timing
            uint32_t pid_update_usec_prev_;

            // Support for LED blink
            Led led_;

            // Safety
            bool was_arming_button_up_;

            // MSP receiver
            MspParser receiver_parser_;
            uint32_t receiver_message_msec_;

            // ---------------------------------------------------------------

    }; // class FlightController

} // namespace hf
