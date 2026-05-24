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

            typedef struct {

                uint32_t timestamp;
                bool armed;
                bool hovering;
                Setpoint setpoint;

            } message_t;

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
                const auto isGyroCalibrated = _imuFilter.isGyroCalibrated;

                // Blink IMU to indicate status
                _led.blink(isGyroCalibrated);

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
                state = EKF::getVehicleState(_ekf);

                // Periodically send telemetry
                if (_telemetryTimer.ready()) {
                    sendTelemetry(state);
                }

                // Run safety checks
                _mode = Safety::updateMode(millis(), state, isGyroCalibrated, 
                        rxRequestedArming, rxMsecPrev, _imuFilter, _mode);

                // Periodically run flying check to get status for EKF
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

            // idle, armed, etc.
            mode_e _mode;

            // Computation
            ImuFilter _imuFilter;
            EKF _ekf;
            FlyingCheck _flyingCheck;

            // Devices
            IMU _imu;
            LED _led = LED(LED_PIN);

            // Timers
            Timer _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
            Timer _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
            Timer _flowdeckTimer = Timer(FLOWDECK_ACQUISITION_RATE_HZ);
            Timer _telemetryTimer = Timer(TELEMETRY_RATE_HZ);

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
