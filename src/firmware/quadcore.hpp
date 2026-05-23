/*
   Hackflight core firmware for quadcopter

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

// Third-party libraries
#include <dshot-teensy4.hpp>  

// Hackflight library

#include <hackflight.h>
#include <datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/flying.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/led.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/profiling.hpp>
#include <firmware/safety.hpp>
#include <firmware/msp/serializer.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

namespace hf {

    class QuadCore {

        private:

            // Arbitrary
            static const uint8_t LED_PIN = 9;

            static constexpr uint32_t TIMEOUT_MSEC = 500;

            // Rate constants
            static constexpr float EKF_PREDICTION_RATE_HZ = 100;
            static constexpr float FLYING_CHECK_RATE_HZ   = 25;
            static constexpr float FLOWDECK_ACQUISITION_RATE_HZ = 100;
            static constexpr float TELEMETRY_RATE_HZ = 50;

        public:

            VehicleState state;
            bool isGyroCalibrated;

            void begin()
            {
                Serial1.begin(115200);

                _imu.begin();
                _motors.begin(); 
                _led.begin(); 
            }

            void getState()
            {
                _led.blink(_imuFilter.isGyroCalibrated);

                const auto imuraw = _imu.read();

                _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
                        _imu.gyroRangeDps(), _imu.accelRangeGs());

                // Run the system dynamics to predict the state forward.
                if (_ekfPredictionTimer.ready()) {
                    _ekf = EKF::predict(_ekf, millis(), _flyingCheck.isFlying); 
                }

                // Faster EKF update with IMU readings
                _ekf = EKF::update(_ekf, _imuFilter.output, millis());

                // Get vehicle state from EKF
                state = EKF::getVehicleState(_ekf);

                // Send telemetry periodically
                if (_telemetryTimer.ready()) {
                    sendTelemetry(state);
                }

                isGyroCalibrated = _imuFilter.isGyroCalibrated;
            } 

            void runMotors(
                    const uint32_t rxMsecPrev,
                    const bool rxRequestedArming,
                    const Setpoint & pidSetpoint)
            {
                // Check receiver timeout
                const auto isArmed =
                    checkTimeout(millis(), rxMsecPrev, rxRequestedArming);

                _mode = Safety::updateMode(state, isArmed, _imuFilter, _mode);

                //_debugger.report(_mode);

                _mixer = Mixer::run(_mixer, pidSetpoint);

                if (_mode != MODE_PANIC) {
                    _motors.run(_mode != MODE_IDLE, _mixer.motorvals);
                }

                if (_flyingCheckTimer.ready()) {
                    _flyingCheck = FlyingCheck::run(
                            _flyingCheck, millis(), _mixer.motorvals, 4);
                }
            }

        private:

            // Computation
            ImuFilter _imuFilter;
            Mixer _mixer;
            EKF _ekf;
            mode_e _mode;
            FlyingCheck _flyingCheck;

            // Devices
            IMU _imu;
            LED _led = LED(LED_PIN);
            DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

            // Timers
            Timer _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
            Timer _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
            Timer _flowdeckTimer = Timer(FLOWDECK_ACQUISITION_RATE_HZ);
            Timer _telemetryTimer = Timer(TELEMETRY_RATE_HZ);

            // Debugging / profiling
            Debugger _debugger;
            Profiler _profiler;

            void sendTelemetry(const VehicleState & state)
            {
                static MspSerializer _serializer;

                _serializer = MspSerializer::serializeFloats(
                        _serializer, MSP_STATE, (float *)&state, 10);

                Serial1.write(
                        MspSerializer::payloadBytes(_serializer),
                        MspSerializer::payloadSize(_serializer));
            }

            static auto checkTimeout(
                    const uint32_t msec_curr,
                    const uint32_t msec_prev,
                    const bool is_armed) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > TIMEOUT_MSEC;

                return timed_out ? false : is_armed;
            } 


    }; // class QuadCore

} // namespace hf
