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
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>
#include <firmware/msp/serializer.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/profiling.hpp>
#include <firmware/receivers/gamepad.hpp>
#include <firmware/receivers/springy.hpp>
#include <firmware/receivers/traditional.hpp>
#include <firmware/timer.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
#include <pidcontrol/hover.hpp>

namespace hf {

    class FC {

        // Constants ---------------------------------------------------------

        private:

            // Voltage sensing
            static constexpr uint8_t VOLTAGE_INPUT_PIN = A9;
            static constexpr float VOLTAGE_SCALEUP = 4.29;

            // LED indicator
            static const uint8_t LED_PIN = 9;
            static constexpr float LED_HEARTBEAT_FREQ_HZ = 0.75;
            static constexpr float LED_FASTBLINK_FREQ_HZ = 3;
            static constexpr uint32_t LED_PULSE_DURATION_MSEC = 50;

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

            void Begin(const bool use_hover_deck=true)
            {
                Serial1.begin(115200);

                imu_.begin();

                pinMode(LED_PIN, OUTPUT); 

                if (use_hover_deck) {
                    zranger_.begin();
                    flow_sensor_.begin();
                }

                mode_ = MODE_IDLE;
            }

            auto Update(
                    const TraditionalReceiver & rx,
                    const float * motor_vals,
                    const uint8_t motor_count) -> Setpoint
            {
                const auto rxdata = rx.data;

                step(rxdata.requested_arming, false, // false = no hover
                        rxdata.timestamp_msec, motor_vals, motor_count);

                const auto rx_setpoint = rxdata.setpoint;

                const auto setpoint = Setpoint(
                        (rx_setpoint.thrust+1)/2, // [-1,+1] => [0,1]
                        rx_setpoint.roll *
                        PositionController::MAX_DEMAND_DEG,
                        rx_setpoint.pitch *
                        PositionController::MAX_DEMAND_DEG, 
                        rx_setpoint.yaw);

                stabilizer_pid_ = StabilizerPidController::run( stabilizer_pid_,
                        is_flying_, getDt(), state_, setpoint);

                sendTelemetry(stabilizer_pid_.setpoint);

                return stabilizer_pid_.setpoint;
            } 

            auto Update(
                    const SpringyReceiver & rx,
                    const float * motor_vals,
                    const uint8_t motor_count) -> Setpoint
            {
                // Run sensor fusion on hover-deck
                AcquireHoverData();

                const auto rxdata = rx.data;

                return update(rxdata.setpoint, rxdata.requested_arming,
                        rxdata.requested_hover, rxdata.timestamp_msec,
                        motor_vals, motor_count);
            } 

            auto Update(
                    GamepadReceiver & gamepad,
                    const float * motor_vals,
                    const uint8_t motor_count) -> Setpoint
            {
                // Run sensor fusion on hover-deck
                AcquireHoverData();

                const auto gpdata = gamepad.data;

                return update(gpdata.setpoint, gpdata.requested_arming,
                        gpdata.requested_hover, gpdata.timestamp_msec,
                        motor_vals, motor_count);
            } 

            void AcquireHoverData()
            {
                // Slower EKF update with range, optical flow
                if (hover_deck_timer_.Ready()) {
                    zranger_filter_ = ZRangerFilter::update(
                            zranger_filter_, zranger_.read());
                    optical_flow_filter_ = OpticalFlowFilter::update(
                            optical_flow_filter_,
                            micros(), flow_sensor_.read());
                    ekf_ = EKF::update(ekf_, zranger_filter_, optical_flow_filter_);
                }
            }

            auto IsSafeToFly() -> bool
            {
                return mode_ != MODE_PANIC;
            }

            auto IsArmed() -> bool
            {
                return mode_ != MODE_IDLE;
            }

        private:

            // Static methods ------------------------------------------------

            static auto updateMode(
                    const uint32_t msecCurr,
                    const VehicleState & state,
                    const bool is_gyro_calibrated,
                    const bool requestedArming,
                    const bool requestedHover,
                    const uint32_t msecPrev,
                    const ImuFilter & imufilt,
                    const Mode mode) -> Mode
            {
                const auto shouldArm = 

                    // Disable arming while gyro is calibrating
                    !is_gyro_calibrated ? false :

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
                    mode == MODE_IDLE && shouldArm && imufilt.is_gyro_calibrated
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

            // Timers
            Timer ekf_prediction_timer_ = Timer(EKF_PREDICTION_RATE_HZ);
            Timer flying_check_timer_ = Timer(FLYING_CHECK_RATE_HZ);
            Timer hover_deck_timer_ = Timer(HOVER_DECK_ACQUISITION_RATE_HZ);
            Timer telemetry_timer_ = Timer(TELEMETRY_RATE_HZ);

            // PID control for stabilize-only
            StabilizerPidController stabilizer_pid_;

            // PID control for hover
            HoverPidController hover_pid_;

            // Telemetry serializer
            MspSerializer telemetry_serializer_;

            // Debugging
            Debugger debugger_;

            // Support for getDt()
            uint32_t usec_prev_;

            // Support for LED blink
            bool is_led_pusing_;
            uint32_t led_pulse_start_;

            // Support for LED blinking
            Timer heartbeat_timer_ = Timer(LED_HEARTBEAT_FREQ_HZ);
            Timer fast_blink_timer_ = Timer(LED_FASTBLINK_FREQ_HZ);

            // Instance methods ---------------------------------------------0

            auto update(
                    const Setpoint & setpoint_in,
                    const bool requested_arming,
                    const bool requested_hover,
                    const uint32_t timestamp_msec,
                    const float * motor_vals,
                    const uint8_t motor_count) -> Setpoint
            {
                runDelayLoop(micros(), CORE_LOOP_HZ);

                step(requested_arming, requested_hover,
                        timestamp_msec, motor_vals, motor_count);

                AcquireHoverData();

                hover_pid_= HoverPidController::run(hover_pid_,
                        getDt(), mode_, state_, setpoint_in);

                const auto setpoint_out = hover_pid_.setpoint;

                sendTelemetry(setpoint_out);

                return setpoint_out;
             } 

            void step(
                    const bool requested_arming,
                    bool requested_hover,
                    const uint32_t timestamp_msec,
                    const float * motor_vals,
                    const uint8_t motor_count)
            {
                // Safely update flight mode
                mode_ = updateMode(millis(), state_,
                        imu_filter_.is_gyro_calibrated, requested_arming,
                        requested_hover, timestamp_msec, imu_filter_, mode_);

                // Periodically run flying check to get status for EKF
                is_flying_ = 

                    mode_ == MODE_IDLE || mode_ == MODE_PANIC  ? false :

                    flying_check_timer_.Ready() ?
                    areMotorsAboveIdle(motor_vals, motor_count) :

                    is_flying_;

                // Blink LED to indicate status
                blinkLed(imu_filter_.is_gyro_calibrated && mode_ != MODE_PANIC);

                // Read the raw IMU data
                const auto imuraw = imu_.read();

                // Filter the raw IMU data
                imu_filter_ = ImuFilter::step(imu_filter_, millis(), imuraw,
                        imu_.gyroRangeDps(), imu_.accelRangeGs());

                // Periodically run the EKF prediction step
                if (ekf_prediction_timer_.Ready()) {
                    ekf_ = EKF::predict(ekf_, millis(), is_flying_); 
                }

                // Do EKF fast-update with IMU readings
                ekf_ = EKF::update(ekf_, imu_filter_.output, millis());

                // Get vehicle state from EKF
                state_ = EKF::getVehicleState(ekf_);
            }

            auto areMotorsAboveIdle(
                    const float * motor_vals,
                    const uint8_t motor_count) -> bool
            {
                auto is_thrust_hover_idle = false;

                for (int i = 0; i < motor_count; ++i) {
                    if (motor_vals[i] > MOTOR_IDLE_MAX) {
                        is_thrust_hover_idle = true;
                        break;
                    }
                }

                const auto msec_curr = millis();

                motor_check_msec_ = is_thrust_hover_idle ? msec_curr :
                    motor_check_msec_;

                return  motor_check_msec_ > 0 &&
                    (msec_curr - motor_check_msec_) <
                    FLYING_HYSTERESIS_THRESHOLD_MSEC;
            }

            void sendTelemetry(const Setpoint & setpoint)
            {
                if (telemetry_timer_.Ready()) {

                    const float data[15] = {
                        (float)mode_,
                        setpoint.thrust, setpoint.roll, setpoint.pitch,
                        setpoint.yaw, state_.dx, state_.dy, state_.z, state_.dz,
                        state_.phi, state_.dphi, state_.theta, state_.dtheta,
                        state_.psi, state_.dpsi
                    };

                    telemetry_serializer_ = MspSerializer::serializeFloats(
                            telemetry_serializer_, MSP_TELEMETRY, data, 15);

                    Serial1.write(
                            MspSerializer::payloadBytes(telemetry_serializer_),
                            MspSerializer::payloadSize(telemetry_serializer_));
                }
            }

            auto getDt() -> float
            {
                const auto usec_curr = micros();      
                const float dt = (usec_curr - usec_prev_)/1000000.0;
                usec_prev_ = usec_curr;

                return dt;
            }

            void blinkLed(const bool isimu__calibrated)
            {
                const auto ready = isimu__calibrated ?
                    heartbeat_timer_.Ready() : fast_blink_timer_.Ready();
                
                if (ready) {
                    digitalWrite(LED_PIN, true);
                    is_led_pusing_ = true;
                    led_pulse_start_ = millis();
                }

                else if (is_led_pusing_) {
                    if (millis() - led_pulse_start_ > LED_PULSE_DURATION_MSEC) {
                        digitalWrite(LED_PIN, false);
                        is_led_pusing_ = false;
                    }
                }
            }

    }; // class FC

} // namespace hf
