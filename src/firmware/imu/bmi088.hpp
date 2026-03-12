/*
   Hackflight for Teensy 4.0 with ELRs receiver

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

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <BMI088.h>

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/estimators/ekf/ekf.hpp>
#include <firmware/imu/new/filter.hpp>

namespace hf {

    class IMU {

        private:

            static const int16_t GYRO_SCALE = 2000;
            static const int16_t ACCEL_SCALE = 24;

            static const uint32_t FREQ_EKF_PREDICTION = 100;

        public:

            bool begin()
            {
                return 

                    okay(gyro.begin()) &&

                    okay(accel.begin()) &&

                    okay(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&

                    okay(gyro.setRange(Bmi088Gyro::RANGE_2000DPS)) &&

                    okay(gyro.pinModeInt3(
                                Bmi088Gyro::PIN_MODE_PUSH_PULL,
                                Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

                    okay(gyro.mapDrdyInt3(true)) &&

                    okay(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&

                    okay(accel.setRange(Bmi088Accel::RANGE_24G));
            }

            auto getVehicleState(const bool isFlying) -> VehicleState
            {
                const auto usec_curr = micros();

                const auto imuraw = read();

                ImuFiltered imufilt = {};
                const bool imuIsCalibrated =
                    _imuFilter.step( usec_curr/1000, imuraw, GYRO_SCALE,
                            ACCEL_SCALE, imufilt);
                (void)imuIsCalibrated; // XXX should rapid-blink LED until IMU calibrated

                _ekf.enqueueImu(&imufilt.gyroDps, &imufilt.accelGs);

                static Timer _timer;

                static bool _didResetEstimation;

                const uint32_t msec_curr = millis();

                if (_didResetEstimation) {
                    _ekf.reset(msec_curr);
                    _didResetEstimation = false;
                }

                // Run the system dynamics to predict the state forward.
                if (_timer.ready(FREQ_EKF_PREDICTION)) {
                    _ekf.predict(msec_curr, isFlying); 
                }

                // Get state estimate from EKF
                const EstimatedState state = _ekf.getStateEstimate(msec_curr);

                // Get angular velocities directly from gyro
                return VehicleState(
                        state.dx,
                        state.dy,
                        state.z,
                        state.dz,
                        state.phi,
                        imufilt.gyroDps.x,
                        state.theta,
                        imufilt.gyroDps.y,
                        state.psi,
                        -imufilt.gyroDps.z); // negate for nose-right positive.y
            }

        private:

            ImuFilter _imuFilter;

            Bmi088Accel accel = Bmi088Accel(Wire, 0x19);
            Bmi088Gyro gyro = Bmi088Gyro(Wire, 0x69);

            EKF _ekf;

            auto read() -> ImuRaw
            {
                gyro.readSensor();

                accel.readSensor();

                return ImuRaw(
                        Vec3Raw(
                            gyro.getGyroX_raw(),
                            gyro.getGyroY_raw(),
                            gyro.getGyroZ_raw()
                            ),
                        Vec3Raw(
                            accel.getAccelX_raw(),
                            accel.getAccelY_raw(),
                            accel.getAccelZ_raw()
                            ));
            }

            static bool okay(const int status)
            {
                return status >= 0;
            }

    };
}
