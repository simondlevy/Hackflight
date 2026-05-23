/*
   Hackflight core firmware

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
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

namespace hf {

    class Core {

        private:

            // Arbitrary
            static const uint8_t LED_PIN = 9;

            // Rate constants
            static constexpr float EKF_PREDICTION_RATE_HZ = 100;
            static constexpr float FLYING_CHECK_RATE_HZ   = 25;
            static constexpr float FLOWDECK_ACQUISITION_RATE_HZ = 100;
            static constexpr float TELEMETRY_RATE_HZ = 50;

        public:

            VehicleState state;

            void begin()
            {
                Serial1.begin(115200);

                _imu.begin();
                _led.begin(); 
            }

            void update(
                    const uint32_t rxMsecPrev, const bool rxRequestedArming,
                    const float * motorvals, const uint8_t motorcount)
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

                // Disable arming while gyro is calibrating
                const auto is_armed = isGyroCalibrated ? rxRequestedArming : false;

                _mode = Safety::updateMode(state, is_armed, millis(),
                        rxMsecPrev, _imuFilter, _mode);

                //_debugger.report(_mode);

                if (_flyingCheckTimer.ready()) {
                    _flyingCheck = FlyingCheck::run(_flyingCheck, millis(),
                            motorvals, motorcount);
                }
            } 

            bool isSafeToFly()
            {
                return _mode != MODE_PANIC;
            }

            bool isArmed()
            {
                return _mode != MODE_IDLE;
            }

        private:

            // Computation
            ImuFilter _imuFilter;
            EKF _ekf;
            mode_e _mode;
            FlyingCheck _flyingCheck;

            // Devices
            IMU _imu;
            LED _led = LED(LED_PIN);

            // Timers
            Timer _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
            Timer _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
            Timer _flowdeckTimer = Timer(FLOWDECK_ACQUISITION_RATE_HZ);
            Timer _telemetryTimer = Timer(TELEMETRY_RATE_HZ);

            // Safety
            bool isGyroCalibrated;

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

    }; // class Core

} // namespace hf
